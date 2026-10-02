`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// sferic_capture.v -- lightning (sferic) event capture with GPS-relative
//                     20 ns timestamps, for a multi-station time-of-arrival net
//
// A lightning stroke arrives at a VLF antenna as a broadband impulse far
// larger than the steady station carriers. This block watches every raw ADC
// sample; when |code - 2048| reaches THRESH it records
//
//   * which GPS second it was in (pps_cnt) and how many 50 MHz clocks had
//     passed since that second's PPS edge (20 ns resolution),
//   * a 64-sample snapshot: 16 samples before the trigger, the trigger sample,
//     47 after it (2.56 us per sample), so a host can interpolate the arrival
//     to a fraction of a sample and cross-correlate stations,
//   * the largest |x| in the snapshot and its sign (the stroke's polarity),
//
// then ignores further triggers for HOLDOFF samples (one stroke = one event).
// Up to 32 events wait in a FIFO; the host reads the oldest one's header and
// snapshot and pops it. Triggers that arrive while the FIFO is full are
// counted in `lost` and dropped.
//
// Readout of the oldest event is through registered outputs (ev_*) and a
// two-samples-per-word snapshot port (snap_addr -> snap_data) that is
// re-read continuously, so a register read sees valid data as long as the
// address was stable for a few clocks -- always true over a UART.
//
// Stations in a network see the same stroke; subtracting their timestamps
// gives the time-of-arrival differences (1 us = 300 m) that locate it.
//-----------------------------------------------------------------------------
module sferic_capture #(
    parameter integer NEV = 32,        // event slots in the FIFO (power of two)
    parameter integer PRE = 16,        // samples kept before the trigger
    parameter integer LEN = 64         // samples per snapshot (power of two)
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire               a_valid,       // one-clock strobe per ADC sample
    input  wire [11:0]        a_code,        // raw ADC code
    input  wire signed [11:0] a_xs,          // code - 2048

    input  wire               pps_rise,      // synchronised GPS PPS edge
    input  wire [31:0]        pps_cnt,       // PPS edges since reset

    input  wire [11:0]        thresh,        // |code - 2048| at or above this triggers; 0 = off
    input  wire [15:0]        holdoff,       // samples ignored after a trigger
    input  wire               ack,           // pop the oldest event (one clock)

    output reg  [31:0]        count,         // events captured since reset
    output reg  [31:0]        lost,          // triggers dropped (FIFO full)
    output reg  [5:0]         nev,           // events waiting

    output reg  [31:0]        ev_pps,        // oldest event: pps_cnt at the trigger
    output reg  [31:0]        ev_clk,        // bit 31 = a PPS had been seen; [25:0] clocks since it
    output reg  [31:0]        ev_info,       // {PRE[7:0], LEN[7:0], 3'b0, polarity, peak[11:0]}
    output reg  [31:0]        ev_seq,        // event number (count at the trigger)

    input  wire [4:0]         snap_addr,     // word 0..31 of the oldest event's snapshot
    output reg  [31:0]        snap_data,     // {4'b0, sample 2k+1, 4'b0, sample 2k}

    output reg                trig           // one-clock strobe, the clock after a trigger sample
);
    localparam integer EW = $clog2(NEV);     // event index bits (5)
    localparam integer SW = $clog2(LEN);     // sample index bits (6)
    localparam integer PW = $clog2(PRE);     // pre-buffer index bits (4)
    localparam [SW-1:0] PRE_S = PRE;
    localparam [7:0]    PRE_8 = PRE;
    localparam [7:0]    LEN_8 = LEN;
    localparam [5:0]    NEV_6 = NEV;

    // ------------------------------------------------ clocks since the last PPS edge
    // Restarts at 1 on the clock that sees the PPS edge, so a sample registered
    // k clocks after that edge reads exactly k (saturates at 2^26-1 = 1.34 s).
    reg [25:0] clk_since_pps;
    reg        pps_seen;
    always @(posedge clk) begin
        if (!rst_n) begin
            clk_since_pps <= 26'd0;
            pps_seen      <= 1'b0;
        end else if (pps_rise) begin
            clk_since_pps <= 26'd1;
            pps_seen      <= 1'b1;
        end else if (clk_since_pps != {26{1'b1}}) begin
            clk_since_pps <= clk_since_pps + 26'd1;
        end
    end

    // ------------------------------------------------ pre-trigger ring of the last PRE samples
    reg [11:0]   pre_buf [0:PRE-1];
    reg [PW-1:0] pre_wp;                     // oldest entry (next to be overwritten)

    // ------------------------------------------------ snapshot memory: NEV slots x LEN samples
    (* ram_style = "block" *) reg [11:0] snap_ram [0:NEV*LEN-1];
    reg              ram_we;
    reg [EW+SW-1:0]  ram_wa;
    reg [11:0]       ram_wd;
    always @(posedge clk) if (ram_we) snap_ram[ram_wa] <= ram_wd;

    // event headers (distributed RAM)
    reg [31:0] h_pps  [0:NEV-1];
    reg [31:0] h_clk  [0:NEV-1];
    reg [31:0] h_info [0:NEV-1];
    reg [31:0] h_seq  [0:NEV-1];

    reg [EW-1:0] wp, rp;

    // ------------------------------------------------ capture state machine
    localparam S_IDLE = 2'd0, S_COPY = 2'd1, S_POST = 2'd2;
    reg [1:0]    state;
    reg [15:0]   hold;                       // samples still to ignore
    reg [PW:0]   copy_i;                     // pre-buffer copy index
    reg [SW-1:0] post_i;                     // next snapshot index to write
    reg [11:0]   trig_code;                  // trigger sample, written to the ring after the copy
    reg [11:0]   peak;
    reg          pol;
    reg [31:0]   t_pps, t_seq;
    reg [25:0]   t_clk;
    reg          t_seen;
    reg          commit;

    wire [11:0] abs_x  = a_xs[11] ? (~a_xs + 12'd1) : a_xs;    // |x|, 2048 for -2048
    wire        armed  = (thresh != 12'd0) && (hold == 16'd0) && (state == S_IDLE);
    wire        hit    = a_valid && armed && (abs_x >= thresh);
    wire        full   = (nev == NEV_6);
    wire [PW-1:0] copy_rd = pre_wp + copy_i[PW-1:0];           // oldest first, wraps mod PRE

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            state  <= S_IDLE;
            hold   <= 16'd0;
            copy_i <= 0;
            post_i <= 0;
            pre_wp <= 0;
            ram_we <= 1'b0;
            ram_wa <= 0;
            ram_wd <= 12'd0;
            wp     <= 0;
            count  <= 32'd0;
            lost   <= 32'd0;
            commit <= 1'b0;
            trig   <= 1'b0;
            peak   <= 12'd0;
            pol    <= 1'b0;
            trig_code <= 12'd0;
            t_pps <= 32'd0; t_seq <= 32'd0; t_clk <= 26'd0; t_seen <= 1'b0;
            for (i = 0; i < PRE; i = i + 1) pre_buf[i] <= 12'd2048;
        end else begin
            ram_we <= 1'b0;
            commit <= 1'b0;
            trig   <= 1'b0;

            if (a_valid && hold != 16'd0) hold <= hold - 16'd1;

            case (state)
                S_IDLE: begin
                    if (hit) begin
                        if (full) begin
                            lost <= lost + 32'd1;
                            hold <= holdoff;
                            pre_buf[pre_wp] <= a_code;
                            pre_wp <= pre_wp + 1'b1;
                        end else begin
                            trig      <= 1'b1;
                            hold      <= holdoff;
                            t_pps     <= pps_cnt;
                            t_clk     <= clk_since_pps;
                            t_seen    <= pps_seen;
                            t_seq     <= count;
                            peak      <= abs_x;
                            pol       <= a_xs[11];
                            trig_code <= a_code;
                            // the trigger sample goes to index PRE now
                            ram_we <= 1'b1;
                            ram_wa <= {wp, PRE_S};
                            ram_wd <= a_code;
                            copy_i <= 0;
                            post_i <= PRE_S + 1'b1;
                            state  <= S_COPY;
                        end
                    end else if (a_valid) begin
                        pre_buf[pre_wp] <= a_code;
                        pre_wp <= pre_wp + 1'b1;
                    end
                end

                // one pre-trigger sample per clock (PRE clocks, well inside one sample period)
                S_COPY: begin
                    if (copy_i < PRE) begin
                        ram_we <= 1'b1;
                        ram_wa <= {wp, {(SW-PW){1'b0}}, copy_i[PW-1:0]};
                        ram_wd <= pre_buf[copy_rd];
                        copy_i <= copy_i + 1'b1;
                    end else begin
                        pre_buf[pre_wp] <= trig_code;        // ring stays continuous
                        pre_wp <= pre_wp + 1'b1;
                        state  <= S_POST;
                    end
                end

                // the next LEN-PRE-1 samples
                S_POST: begin
                    if (a_valid) begin
                        pre_buf[pre_wp] <= a_code;
                        pre_wp <= pre_wp + 1'b1;
                        ram_we <= 1'b1;
                        ram_wa <= {wp, post_i};
                        ram_wd <= a_code;
                        if (abs_x > peak) begin
                            peak <= abs_x;
                            pol  <= a_xs[11];
                        end
                        if (post_i == LEN - 1) begin
                            commit <= 1'b1;
                            state  <= S_IDLE;
                        end
                        post_i <= post_i + 1'b1;
                    end
                end

                default: state <= S_IDLE;
            endcase

            // commit: header written the clock after the last sample (peak is final)
            if (commit) begin
                h_pps[wp]  <= t_pps;
                h_clk[wp]  <= {t_seen, 5'd0, t_clk};
                h_info[wp] <= {PRE_8, LEN_8, 3'b000, pol, peak};
                h_seq[wp]  <= t_seq;
                wp         <= wp + 1'b1;
                count      <= count + 32'd1;
            end
        end
    end

    // ------------------------------------------------ FIFO occupancy and the read side
    wire pop = ack && (nev != 6'd0);
    always @(posedge clk) begin
        if (!rst_n) begin
            nev <= 6'd0;
            rp  <= 0;
        end else begin
            nev <= nev + {5'd0, commit} - {5'd0, pop};
            if (pop) rp <= rp + 1'b1;
        end
    end

    // oldest event's header, re-read every clock
    always @(posedge clk) begin
        ev_pps  <= h_pps[rp];
        ev_clk  <= h_clk[rp];
        ev_info <= h_info[rp];
        ev_seq  <= h_seq[rp];
    end

    // oldest event's snapshot: two samples per word, read alternately
    reg        rd_ph;
    reg [11:0] rd_q;
    always @(posedge clk) begin
        if (!rst_n) begin
            rd_ph     <= 1'b0;
            rd_q      <= 12'd0;
            snap_data <= 32'd0;
        end else begin
            rd_ph <= ~rd_ph;
            rd_q  <= snap_ram[{rp, snap_addr, rd_ph}];
            // rd_q now holds the half addressed one clock ago, i.e. half ~rd_ph
            if (rd_ph) snap_data[11:0]  <= rd_q;
            else       snap_data[27:16] <= rd_q;
            snap_data[15:12] <= 4'd0;
            snap_data[31:28] <= 4'd0;
        end
    end

endmodule
