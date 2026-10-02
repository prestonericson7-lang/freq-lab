`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// vlf_top.v -- PZ7020-StarLite 8-station VLF receiver for solar-flare (SID) work
//
// Navy VLF transmitters (NLK, NPM, NAA, ...) bounce between the ground and the
// bottom of the ionosphere. When a solar flare's X-rays hit the dayside
// ionosphere, the reflection height drops within minutes and every station's
// received amplitude jumps or dips -- a Sudden Ionospheric Disturbance (SID).
// This design measures the amplitude of 8 stations at once, once per second,
// aligned to GPS seconds if a GPS PPS is connected.
//
//   antenna -> preamp (0..1 V) -> XADC VAUX1 (390,625 S/s, 12 bit)
//                                    |
//              +---------------------+---------------------+
//              v                     v                     v
//         vlf_channel 0  ...    vlf_channel 7         input statistics
//         (NCO, mixer, CIC,     one per station        (RMS, DC, clipping)
//          I^2+Q^2 per window)
//              |                                           |
//              +------------> cmd_regs_vlf (UART) <--------+
//
// Pins (JM1, all 3.3 V LVCMOS bank -- same places as the other freq-lab designs):
//   JM1-6   E17  VAUX1P   preamp output, 0 .. 1.0 V ONLY (bias at 0.5 V)
//   JM1-8   D18  VAUX1N   preamp ground at the source
//   JM1-14  B19  UART TX  -> USB-serial adapter RX (3.3 V)
//   JM1-16  A20  UART RX  <- USB-serial adapter TX (3.3 V)
//   JM1-18  C20  GPS PPS  1PPS from a 3.3 V GPS module (optional; pulled down)
//   JM1-5   H16  fan PWM, JM1-7 H17 fan tach (as before)
//   LED R19 heartbeat, LED V13 blinks once per window, solid = input clipped
//   KEY1 G14 held = fan 100 %
//
// Windows: with CTRL.PPS_MODE = 1 and a PPS present, each window runs from one
// PPS edge to the next (GPS seconds). Without PPS (or PPS_MODE = 0) windows are
// WIN samples long (default 390625 = 1 s by the 50 MHz crystal). If PPS stops,
// a window closes after 1.125 x WIN samples and after 2 x WIN samples without
// PPS the design falls back to WIN-sample windows.
//-----------------------------------------------------------------------------
module vlf_top #(
    parameter integer CLK_HZ           = 50_000_000,
    parameter integer BAUD             = 115_200,
    parameter integer FAN_DUTY_DEFAULT = 60
)(
    input  wire clk_50m,
    output wire led_hb,
    output wire led_act,
    input  wire key_n,

    output wire fan_pwm_o,
    input  wire fan_tach,

    input  wire vauxp1,
    input  wire vauxn1,

    input  wire gps_pps,
    output wire uart_tx,
    input  wire uart_rx
);
    localparam integer NCH   = 8;
    localparam integer LOG2R = 10;                 // CIC decimation 1024
    localparam integer CIC_W = 27 + 3 * LOG2R;     // 57
    localparam integer CIC_S = CIC_W - 24;         // 33

    wire clk = clk_50m;

    // ------------------------------------------------ reset: 2^16 clocks (1.3 ms) after configuration
    localparam integer RST_TICKS = CLK_HZ / 1000;
    reg [$clog2(RST_TICKS):0] rst_cnt = 0;
    wire rst_n = rst_cnt[$clog2(RST_TICKS)];
    always @(posedge clk) if (!rst_n) rst_cnt <= rst_cnt + 1'b1;

    // ------------------------------------------------ heartbeat
    reg [$clog2(CLK_HZ/2):0] hb_cnt = 0;
    reg hb = 0;
    always @(posedge clk) begin
        if (!rst_n) begin hb_cnt <= 0; hb <= 1'b0; end
        else if (hb_cnt == CLK_HZ/2 - 1) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1'b1;
    end
    assign led_hb = hb;

    // ------------------------------------------------ register file outputs
    wire                run;
    wire                pps_mode;
    wire [23:0]         win_len;
    wire [NCH*32-1:0]   ftw_bus;
    wire [7:0]          fan_duty;

    // ------------------------------------------------ fan
    wire [15:0] fan_rpm;
    fan_pwm #(.CLK_HZ(CLK_HZ)) u_fan (
        .clk(clk), .rst_n(rst_n),
        .duty_pct((!key_n) ? 8'd100 : fan_duty),
        .fan_pwm(fan_pwm_o), .fan_tach(fan_tach), .rpm(fan_rpm));

    // ------------------------------------------------ XADC, VAUX1, 390,625 S/s
    wire [11:0] adc_code;
    wire        adc_stb;
    xadc_vaux1 u_xadc (
        .clk(clk), .rst(~rst_n),
        .vauxp1(vauxp1), .vauxn1(vauxn1),
        .data(adc_code), .stb(adc_stb));

    // ------------------------------------------------ GPS PPS
    reg [2:0] pps_sync = 3'b000;
    always @(posedge clk) pps_sync <= {pps_sync[1:0], gps_pps};
    wire pps_rise = (pps_sync[2:1] == 2'b01);

    reg        pps_pend;            // a PPS edge arrived since the last sample
    reg [25:0] pps_since;           // samples since the last PPS (saturating)
    reg [31:0] pps_cnt;             // PPS edges since reset

    // ------------------------------------------------ window decision (at adc_stb)
    reg  [31:0] wcnt;               // samples already in the current window
    reg  [LOG2R-1:0] dec_cnt;       // free-running sample count mod 1024

    wire [24:0] win_to    = {1'b0, win_len} + {4'b0000, win_len[23:3]};   // 1.125 x WIN
    wire        pps_alive = pps_pend || (pps_since < {1'b0, win_len, 1'b0});
    wire        use_pps   = pps_mode && pps_alive;
    wire        hit_len   = (wcnt >= {8'd0, win_len});
    wire        hit_to    = (wcnt >= {7'd0, win_to});
    wire        cl_pps    = run &&  use_pps &&  pps_pend;
    wire        cl_to     = run &&  use_pps && !pps_pend && hit_to;
    wire        cl_len    = run && !use_pps && hit_len;
    wire        close_now = cl_pps || cl_to || cl_len;

    // stage-A sample registers shared by the channels and the statistics
    reg               a_valid;
    reg        [11:0] a_code;
    reg signed [11:0] a_xs;
    reg               a_dec, a_close, a_close_pps, a_close_to, a_run;

    always @(posedge clk) begin
        if (!rst_n) begin
            pps_pend    <= 1'b0;
            pps_since   <= {26{1'b1}};
            pps_cnt     <= 32'd0;
            wcnt        <= 32'd0;
            dec_cnt     <= {LOG2R{1'b0}};
            a_valid     <= 1'b0;
            a_code      <= 12'd2048;
            a_xs        <= 12'sd0;
            a_dec       <= 1'b0;
            a_close     <= 1'b0;
            a_close_pps <= 1'b0;
            a_close_to  <= 1'b0;
            a_run       <= 1'b0;
        end else begin
            a_valid <= 1'b0;
            if (pps_rise) pps_cnt <= pps_cnt + 32'd1;

            if (adc_stb) begin
                a_valid     <= 1'b1;
                a_code      <= adc_code;
                a_xs        <= $signed({~adc_code[11], adc_code[10:0]});   // code - 2048
                a_dec       <= (dec_cnt == {LOG2R{1'b1}});
                dec_cnt     <= dec_cnt + 1'b1;
                a_close     <= close_now;
                a_close_pps <= cl_pps;
                a_close_to  <= cl_to;
                a_run       <= run;

                if (!run)          wcnt <= 32'd0;
                else if (close_now) wcnt <= 32'd1;
                else               wcnt <= wcnt + 32'd1;

                pps_pend  <= pps_rise;                  // consumed; an edge on this clock stays pending
                pps_since <= pps_pend ? 26'd1
                           : ((pps_since == {26{1'b1}}) ? pps_since : pps_since + 26'd1);
            end else if (pps_rise) begin
                pps_pend <= 1'b1;
            end
        end
    end

    // ------------------------------------------------ XADC sample rate vs the crystal
    // clocks between sample strobes 65536 apart (nominal 65536 x 128 = 8388608).
    // The host computes fs = 65536 * 50 MHz / FS_CLKS and tunes the channels with
    // it, so the station frequencies never depend on the XADC timing assumption.
    reg [31:0] cc, cc_last, fs_clks;
    reg [15:0] fsm_samp;
    reg        fsm_primed;
    always @(posedge clk) begin
        if (!rst_n) begin
            cc <= 32'd0; cc_last <= 32'd0; fs_clks <= 32'd0;
            fsm_samp <= 16'd0; fsm_primed <= 1'b0;
        end else begin
            cc <= cc + 32'd1;
            if (adc_stb) begin
                fsm_samp <= fsm_samp + 16'd1;
                if (fsm_samp == 16'd0) begin
                    cc_last <= cc;
                    if (fsm_primed) fs_clks <= cc - cc_last;
                    fsm_primed <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------ channels
    wire [NCH*64-1:0] w_pow_bus;
    wire [NCH*48-1:0] w_pmax_bus;
    wire [NCH-1:0]    iq_valid_bus;
    wire [NCH*24-1:0] dbg_i_bus, dbg_q_bus;     // simulation visibility only

    genvar k;
    generate
        for (k = 0; k < NCH; k = k + 1) begin : g_ch
            vlf_channel #(.W(CIC_W), .SH(CIC_S)) u_ch (
                .clk(clk), .rst_n(rst_n),
                .ftw(ftw_bus[k*32 +: 32]),
                .a_valid(a_valid), .a_xs(a_xs), .a_dec(a_dec),
                .a_close(a_close), .a_run(a_run),
                .win_pow(w_pow_bus[k*64 +: 64]),
                .win_pmax(w_pmax_bus[k*48 +: 48]),
                .iq_valid(iq_valid_bus[k]),
                .i_out(dbg_i_bus[k*24 +: 24]),
                .q_out(dbg_q_bus[k*24 +: 24]));
        end
    endgenerate

    // ------------------------------------------------ input statistics per window
    wire        a_clip = (a_code == 12'd0) || (a_code == 12'd4095);
    wire [23:0] a_sq   = a_xs * a_xs;           // <= 2^22, as unsigned

    reg [31:0] s_n, s_clip, s_ndec;
    reg [63:0] s_sq;
    reg [63:0] s_sum;
    reg [11:0] s_min, s_max;
    reg        s_start_pps;

    reg [31:0] seq;                             // finished windows
    reg [31:0] w_n, w_clip, w_ndec;
    reg [63:0] w_sq, w_sum;
    reg [11:0] w_min, w_max;
    reg        w_end_pps, w_start_pps, w_end_to;

    always @(posedge clk) begin
        if (!rst_n) begin
            s_n <= 0; s_clip <= 0; s_ndec <= 0; s_sq <= 0; s_sum <= 0;
            s_min <= 12'hFFF; s_max <= 12'h000; s_start_pps <= 1'b0;
            seq <= 0;
            w_n <= 0; w_clip <= 0; w_ndec <= 0; w_sq <= 0; w_sum <= 0;
            w_min <= 12'h000; w_max <= 12'h000;
            w_end_pps <= 1'b0; w_start_pps <= 1'b0; w_end_to <= 1'b0;
        end else if (a_valid) begin
            if (!a_run) begin
                s_n <= 0; s_clip <= 0; s_ndec <= 0; s_sq <= 0; s_sum <= 0;
                s_min <= 12'hFFF; s_max <= 12'h000; s_start_pps <= 1'b0;
            end else if (a_close) begin
                w_n         <= s_n;
                w_clip      <= s_clip;
                w_ndec      <= s_ndec;
                w_sq        <= s_sq;
                w_sum       <= s_sum;
                w_min       <= s_min;
                w_max       <= s_max;
                w_start_pps <= s_start_pps;
                w_end_pps   <= a_close_pps;
                w_end_to    <= a_close_to;
                seq         <= seq + 32'd1;
                s_n         <= 32'd1;
                s_clip      <= {31'd0, a_clip};
                s_ndec      <= {31'd0, a_dec};
                s_sq        <= {40'd0, a_sq};
                s_sum       <= {{52{a_xs[11]}}, a_xs};
                s_min       <= a_code;
                s_max       <= a_code;
                s_start_pps <= a_close_pps;
            end else begin
                s_n    <= s_n + 32'd1;
                s_clip <= s_clip + {31'd0, a_clip};
                s_ndec <= s_ndec + {31'd0, a_dec};
                s_sq   <= s_sq + {40'd0, a_sq};
                s_sum  <= s_sum + {{52{a_xs[11]}}, a_xs};
                if (a_code < s_min) s_min <= a_code;
                if (a_code > s_max) s_max <= a_code;
            end
        end
    end

    // ------------------------------------------------ activity LED
    // blink 50 ms per finished window; solid while the last window clipped
    localparam integer BLINK = CLK_HZ / 20;
    reg [$clog2(BLINK+1)-1:0] blink_cnt;
    reg                       clip_last;
    always @(posedge clk) begin
        if (!rst_n) begin
            blink_cnt <= 0;
            clip_last <= 1'b0;
        end else if (a_valid && a_close) begin
            blink_cnt <= BLINK;
            clip_last <= (s_clip != 32'd0);
        end else if (blink_cnt != 0) begin
            blink_cnt <= blink_cnt - 1'b1;
        end
    end
    assign led_act = clip_last | (blink_cnt != 0);

    // ------------------------------------------------ register file + UART
    cmd_regs_vlf #(.CLKS_PER_BIT(CLK_HZ / BAUD), .FAN_DUTY_DEFAULT(FAN_DUTY_DEFAULT)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .rx(uart_rx), .tx(uart_tx),
        .run(run), .pps_mode(pps_mode), .win_len(win_len),
        .ftw_bus(ftw_bus), .fan_duty(fan_duty),
        .seq(seq), .pps_alive(pps_alive),
        .last_end_pps(w_end_pps), .last_end_to(w_end_to),
        .w_n(w_n), .w_ndec(w_ndec), .w_clip(w_clip),
        .w_flags({w_end_to, w_start_pps, w_end_pps}),
        .w_min(w_min), .w_max(w_max),
        .w_sq(w_sq), .w_sum(w_sum),
        .w_pow_bus(w_pow_bus), .w_pmax_bus(w_pmax_bus),
        .adc_live(adc_code), .pps_cnt(pps_cnt), .fan_rpm(fan_rpm), .fs_clks(fs_clks));

endmodule
