`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// vlf_channel.v -- one narrowband VLF receiver channel (digital down-converter)
//
//   ADC sample --> x NCO cos --> 3-stage CIC, /1024 --> I --+
//              \-> x NCO sin --> 3-stage CIC, /1024 --> Q --+--> I^2+Q^2 --> sum per window
//
// Input rate 390,625 S/s (one sample every 128 clocks at 50 MHz).
// NCO     : 32-bit phase accumulator, f = ftw * 390625 / 2^32 (0.09 mHz steps).
//           12-bit phase index into a quarter-wave table (spurs about -72 dBc).
// Mixer   : 12-bit signed sample x 16-bit signed LO -> |product| < 2^26.
// CIC     : N = 3, R = 1024, M = 1. Output rate 381.47 Hz. Passband -3 dB at
//           +/-100 Hz, first null at +/-381 Hz, -80 dB at 400 Hz offset.
//           Register width W = 27 + 3*10 = 57 bits (no overflow possible:
//           |out| <= 2^26 * 2^30 < 2^56).
// Output  : 24-bit I/Q = rounded [56:33]. A tone of amplitude A ADC counts at
//           the channel frequency gives sqrt(I^2+Q^2) = A * 32767/16.
// Power   : P = I^2 + Q^2 (48 bits) accumulated per window; maximum P per
//           window kept too (lightning sferics show up there).
//
// Pipeline (stage letters; every stage fires once per sample or per decimation):
//   A  sample in       : NCO step, ROM address
//   B  ROM data        : apply sign -> signed LO
//   C  multiply
//   D  integrate       : decimation capture of the third integrator
//   E,F,G combs        (decimation samples only)
//   H  round to 24 bit : iq_valid
//   I  squares
//   J  I^2 + Q^2
//   K  accumulate
// A..K is 10 clocks; samples are 128 clocks apart, so a decimated output is
// always accumulated before the next sample (and before the next window close).
//
// Window rules (the same in vlf_top's sample statistics):
//   a_close on sample n : results of samples < n are latched, sample n starts
//                         the new window. A decimation at sample n belongs to
//                         the new window.
//   a_run = 0          : accumulators held at zero; decimations of such samples
//                         are not accumulated (the CIC itself keeps running).
//-----------------------------------------------------------------------------
module vlf_channel #(
    parameter integer W  = 57,              // CIC register width
    parameter integer SH = 33               // output slice LSB (W - 24)
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire [31:0]        ftw,

    input  wire               a_valid,      // one-clock strobe per sample
    input  wire signed [11:0] a_xs,         // ADC code - 2048
    input  wire               a_dec,        // last sample of a decimation period
    input  wire               a_close,      // this sample starts a new window
    input  wire               a_run,        // accumulate (CTRL.RUN at this sample)

    output reg  [63:0]        win_pow,      // sum of I^2+Q^2, last finished window
    output reg  [47:0]        win_pmax,     // max   I^2+Q^2, last finished window

    output reg                iq_valid,     // debug/test: new decimated output
    output reg  signed [23:0] i_out,
    output reg  signed [23:0] q_out
);
    localparam [W-1:0] RND = {{(W-SH){1'b0}}, 1'b1, {(SH-1){1'b0}}};   // 2^(SH-1)

    // ------------------------------------------------ A: NCO + ROM address
    reg  [31:0] phase;
    wire [11:0] idx    = phase[31:20];
    wire [1:0]  quad   = idx[11:10];
    wire [9:0]  a_lo   = idx[9:0];
    wire [9:0]  addr_s = quad[0] ? ~a_lo : a_lo;          // sin(idx)
    wire [9:0]  addr_c = quad[0] ?  a_lo : ~a_lo;         // cos(idx) = sin(idx + 1024)
    wire        neg_s  = quad[1];
    wire        neg_c  = quad[1] ^ quad[0];

    wire [14:0] rom_s, rom_c;
    sine_qrom u_rom (
        .clk(clk), .en(a_valid),
        .addr_a(addr_s), .addr_b(addr_c),
        .data_a(rom_s),  .data_b(rom_c));

    reg               b_valid, b_dec, b_run, b_neg_s, b_neg_c;
    reg signed [11:0] b_xs;

    always @(posedge clk) begin
        if (!rst_n) begin
            phase   <= 32'd0;
            b_valid <= 1'b0;
            b_dec   <= 1'b0;
            b_run   <= 1'b0;
            b_neg_s <= 1'b0;
            b_neg_c <= 1'b0;
            b_xs    <= 12'sd0;
        end else begin
            b_valid <= a_valid;
            if (a_valid) begin
                phase   <= phase + ftw;
                b_xs    <= a_xs;
                b_dec   <= a_dec;
                b_run   <= a_run;
                b_neg_s <= neg_s;
                b_neg_c <= neg_c;
            end
        end
    end

    // ------------------------------------------------ B: signed LO
    reg               c_valid, c_dec, c_run;
    reg signed [11:0] c_xs;
    reg signed [15:0] c_lo_s, c_lo_c;

    wire signed [15:0] rom_s_p = $signed({1'b0, rom_s});
    wire signed [15:0] rom_c_p = $signed({1'b0, rom_c});

    always @(posedge clk) begin
        if (!rst_n) begin
            c_valid <= 1'b0;
            c_dec   <= 1'b0;
            c_run   <= 1'b0;
            c_xs    <= 12'sd0;
            c_lo_s  <= 16'sd0;
            c_lo_c  <= 16'sd0;
        end else begin
            c_valid <= b_valid;
            if (b_valid) begin
                c_xs   <= b_xs;
                c_dec  <= b_dec;
                c_run  <= b_run;
                c_lo_s <= b_neg_s ? -rom_s_p : rom_s_p;
                c_lo_c <= b_neg_c ? -rom_c_p : rom_c_p;
            end
        end
    end

    // ------------------------------------------------ C: mixers
    reg               d_valid, d_dec, d_run;
    reg signed [27:0] d_pi, d_pq;

    always @(posedge clk) begin
        if (!rst_n) begin
            d_valid <= 1'b0;
            d_dec   <= 1'b0;
            d_run   <= 1'b0;
            d_pi    <= 28'sd0;
            d_pq    <= 28'sd0;
        end else begin
            d_valid <= c_valid;
            if (c_valid) begin
                d_pi  <= c_xs * c_lo_c;          // I = x * cos
                d_pq  <= c_xs * c_lo_s;          // Q = x * sin
                d_dec <= c_dec;
                d_run <= c_run;
            end
        end
    end

    // ------------------------------------------------ D: integrators + decimation
    reg signed [W-1:0] ii1, ii2, ii3, qi1, qi2, qi3;
    reg signed [W-1:0] ic0, qc0;
    reg                e_valid, e_run;

    wire signed [W-1:0] pi_ext = {{(W-28){d_pi[27]}}, d_pi};
    wire signed [W-1:0] pq_ext = {{(W-28){d_pq[27]}}, d_pq};

    always @(posedge clk) begin
        if (!rst_n) begin
            ii1 <= {W{1'b0}}; ii2 <= {W{1'b0}}; ii3 <= {W{1'b0}};
            qi1 <= {W{1'b0}}; qi2 <= {W{1'b0}}; qi3 <= {W{1'b0}};
            ic0 <= {W{1'b0}}; qc0 <= {W{1'b0}};
            e_valid <= 1'b0;
            e_run   <= 1'b0;
        end else begin
            e_valid <= 1'b0;
            if (d_valid) begin
                ii1 <= ii1 + pi_ext;
                ii2 <= ii2 + ii1;
                ii3 <= ii3 + ii2;
                qi1 <= qi1 + pq_ext;
                qi2 <= qi2 + qi1;
                qi3 <= qi3 + qi2;
                if (d_dec) begin
                    ic0     <= ii3;              // value before this sample's update
                    qc0     <= qi3;
                    e_valid <= 1'b1;
                    e_run   <= d_run;
                end
            end
        end
    end

    // ------------------------------------------------ E, F, G: combs (decimated rate)
    reg signed [W-1:0] iz0, iz1, iz2, qz0, qz1, qz2;
    reg signed [W-1:0] iy1, iy2, iy3, qy1, qy2, qy3;
    reg                f_valid, g_valid, h_valid;
    reg                f_run, g_run, h_run;

    always @(posedge clk) begin
        if (!rst_n) begin
            iz0 <= {W{1'b0}}; iz1 <= {W{1'b0}}; iz2 <= {W{1'b0}};
            qz0 <= {W{1'b0}}; qz1 <= {W{1'b0}}; qz2 <= {W{1'b0}};
            iy1 <= {W{1'b0}}; iy2 <= {W{1'b0}}; iy3 <= {W{1'b0}};
            qy1 <= {W{1'b0}}; qy2 <= {W{1'b0}}; qy3 <= {W{1'b0}};
            f_valid <= 1'b0; g_valid <= 1'b0; h_valid <= 1'b0;
            f_run   <= 1'b0; g_run   <= 1'b0; h_run   <= 1'b0;
        end else begin
            f_valid <= e_valid;
            g_valid <= f_valid;
            h_valid <= g_valid;
            f_run   <= e_run;
            g_run   <= f_run;
            h_run   <= g_run;
            if (e_valid) begin
                iy1 <= ic0 - iz0;  iz0 <= ic0;
                qy1 <= qc0 - qz0;  qz0 <= qc0;
            end
            if (f_valid) begin
                iy2 <= iy1 - iz1;  iz1 <= iy1;
                qy2 <= qy1 - qz1;  qz1 <= qy1;
            end
            if (g_valid) begin
                iy3 <= iy2 - iz2;  iz2 <= iy2;
                qy3 <= qy2 - qz2;  qz2 <= qy2;
            end
        end
    end

    // ------------------------------------------------ H: round to 24 bits
    wire signed [W-1:0] iy3_r = iy3 + RND;
    wire signed [W-1:0] qy3_r = qy3 + RND;
    reg                 i_run;

    always @(posedge clk) begin
        if (!rst_n) begin
            iq_valid <= 1'b0;
            i_run    <= 1'b0;
            i_out    <= 24'sd0;
            q_out    <= 24'sd0;
        end else begin
            iq_valid <= h_valid;
            i_run    <= h_run;
            if (h_valid) begin
                i_out <= iy3_r[W-1:SH];
                q_out <= qy3_r[W-1:SH];
            end
        end
    end

    // ------------------------------------------------ I, J: power
    reg signed [47:0] isq, qsq;
    reg               j_valid, j_run, k_valid, k_run;
    reg        [47:0] pwr;

    always @(posedge clk) begin
        if (!rst_n) begin
            isq <= 48'sd0; qsq <= 48'sd0; pwr <= 48'd0;
            j_valid <= 1'b0; j_run <= 1'b0;
            k_valid <= 1'b0; k_run <= 1'b0;
        end else begin
            j_valid <= iq_valid;
            j_run   <= i_run;
            k_valid <= j_valid;
            k_run   <= j_run;
            if (iq_valid) begin
                isq <= i_out * i_out;
                qsq <= q_out * q_out;
            end
            if (j_valid)
                pwr <= isq[47:0] + qsq[47:0];       // both >= 0, sum <= 2^47
        end
    end

    // ------------------------------------------------ K: window accumulators
    reg [63:0] acc;
    reg [47:0] pmax;

    wire       closing = a_valid && a_close;
    wire       adding  = k_valid && k_run;
    wire [47:0] pmax_add = (pwr > pmax) ? pwr : pmax;

    always @(posedge clk) begin
        if (!rst_n) begin
            acc      <= 64'd0;
            pmax     <= 48'd0;
            win_pow  <= 64'd0;
            win_pmax <= 48'd0;
        end else if (closing) begin
            // an in-flight output always comes from an earlier sample -> old window
            win_pow  <= adding ? acc + {16'd0, pwr} : acc;
            win_pmax <= adding ? pmax_add : pmax;
            acc      <= 64'd0;
            pmax     <= 48'd0;
        end else if (a_valid && !a_run) begin
            acc      <= 64'd0;
            pmax     <= 48'd0;
        end else if (adding) begin
            acc      <= acc + {16'd0, pwr};
            pmax     <= pmax_add;
        end
    end

endmodule
