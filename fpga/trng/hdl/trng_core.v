`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// trng_core.v -- the deterministic TRNG pipeline after the noise source:
//   raw bits --> SP 800-90B health tests (RCT, APT) on the RAW stream
//            --> von Neumann debiaser
//            --> LFSR conditioning (XOR into a maximal-length LFSR)
//            --> 32-bit packer --> output word with a strobe
//
// All of this is deterministic and is verified bit-for-bit by sim/check_trng.py.
//
// SP 800-90B health tests (run continuously on the raw noise-source output):
//   RCT (Repetition Count Test): if the same raw value repeats rct_cutoff times
//     in a row, raise rct_fail (a stuck or oscillating source).
//   APT (Adaptive Proportion Test): over a window of apt_window raw bits, count
//     how many equal the window's first bit; if that count >= apt_cutoff, raise
//     apt_fail (a biased source). A new window starts every apt_window bits.
//   Both latch a sticky alarm (cleared by a CTRL write) and bump a fail counter.
//   While an alarm is latched, conditioned output is withheld (no words packed),
//   so biased/stuck entropy never reaches the host.
//
// von Neumann debiaser: takes raw bits in pairs. 01 -> output 0, 10 -> output 1,
//   00 and 11 -> output nothing. Removes first-order bias at the cost of rate.
//
// LFSR conditioning: each debiased bit is XORed into bit 0 of a 32-bit
//   maximal-length LFSR (x^32 + x^22 + x^2 + x + 1) which is clocked every
//   debiased bit; 32 debiased bits -> one output word. This whitens residual
//   correlation; it is a conditioning step, not the entropy itself.
//-----------------------------------------------------------------------------
module trng_core (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        raw_bit,
    input  wire        raw_stb,

    input  wire        clear_alarms,       // one-clock: clear sticky health alarms
    input  wire [5:0]  rct_cutoff,         // repetition-count cutoff (e.g. 20..40)
    input  wire [15:0] apt_window,         // APT window length (e.g. 1024)
    input  wire [15:0] apt_cutoff,         // APT cutoff count (e.g. 660)

    output reg  [31:0] word,               // a fresh 32-bit random word
    output reg         word_stb,

    output reg         rct_fail,           // sticky alarms
    output reg         apt_fail,
    output reg  [31:0] raw_count,          // raw bits seen
    output reg  [31:0] vn_count,           // von Neumann output bits
    output reg  [31:0] rct_fail_count,
    output reg  [31:0] apt_fail_count
);
    wire healthy = ~rct_fail & ~apt_fail;

    // ---------------- RCT ----------------
    reg        rct_have;
    reg        rct_last;
    reg [15:0] rct_run;
    always @(posedge clk) begin
        if (!rst_n) begin
            rct_have <= 1'b0; rct_last <= 1'b0; rct_run <= 16'd1;
            rct_fail <= 1'b0; rct_fail_count <= 32'd0;
        end else begin
            if (clear_alarms) rct_fail <= 1'b0;
            if (raw_stb) begin
                if (!rct_have) begin
                    rct_have <= 1'b1; rct_last <= raw_bit; rct_run <= 16'd1;
                end else if (raw_bit == rct_last) begin
                    rct_run <= rct_run + 16'd1;
                    if (rct_run + 16'd1 >= {10'd0, rct_cutoff}) begin
                        rct_fail <= 1'b1;
                        rct_fail_count <= rct_fail_count + 32'd1;
                        rct_run <= 16'd1;            // re-arm so one stuck burst = one count
                        rct_have <= 1'b0;
                    end
                end else begin
                    rct_last <= raw_bit; rct_run <= 16'd1;
                end
            end
        end
    end

    // ---------------- APT ----------------
    reg        apt_have;
    reg        apt_ref;
    reg [15:0] apt_n;        // samples into the current window
    reg [15:0] apt_cnt;      // count equal to apt_ref
    wire [15:0] apt_nn = apt_n   + 16'd1;
    wire [15:0] apt_cc = apt_cnt + ((raw_bit == apt_ref) ? 16'd1 : 16'd0);
    always @(posedge clk) begin
        if (!rst_n) begin
            apt_have <= 1'b0; apt_ref <= 1'b0; apt_n <= 16'd0; apt_cnt <= 16'd0;
            apt_fail <= 1'b0; apt_fail_count <= 32'd0;
        end else begin
            if (clear_alarms) apt_fail <= 1'b0;
            if (raw_stb) begin
                if (!apt_have) begin
                    apt_have <= 1'b1; apt_ref <= raw_bit; apt_n <= 16'd1; apt_cnt <= 16'd1;
                end else begin
                    if (apt_cc >= apt_cutoff) begin
                        apt_fail <= 1'b1;
                        apt_fail_count <= apt_fail_count + 32'd1;
                    end
                    if (apt_nn >= apt_window) begin   // window done: start a new one
                        apt_have <= 1'b0; apt_n <= 16'd0; apt_cnt <= 16'd0;
                    end else begin
                        apt_n <= apt_nn; apt_cnt <= apt_cc;
                    end
                end
            end
        end
    end

    // ---------------- raw counter ----------------
    always @(posedge clk) begin
        if (!rst_n) raw_count <= 32'd0;
        else if (raw_stb) raw_count <= raw_count + 32'd1;
    end

    // ---------------- von Neumann ----------------
    reg       vn_have;      // holding the first bit of a pair
    reg       vn_first;
    reg       vn_bit;
    reg       vn_stb;
    always @(posedge clk) begin
        if (!rst_n) begin vn_have <= 1'b0; vn_first <= 1'b0; vn_bit <= 1'b0; vn_stb <= 1'b0; vn_count <= 32'd0; end
        else begin
            vn_stb <= 1'b0;
            if (raw_stb && healthy) begin
                if (!vn_have) begin vn_have <= 1'b1; vn_first <= raw_bit; end
                else begin
                    vn_have <= 1'b0;
                    if (vn_first != raw_bit) begin    // 01 -> 0, 10 -> 1  (output the first bit)
                        vn_bit <= vn_first;
                        vn_stb <= 1'b1;
                        vn_count <= vn_count + 32'd1;
                    end
                end
            end
        end
    end

    // ---------------- LFSR conditioning + 32-bit packer ----------------
    reg [31:0] lfsr;
    reg [5:0]  bitcnt;
    always @(posedge clk) begin
        if (!rst_n) begin
            lfsr <= 32'hDEADBEEF; bitcnt <= 6'd0; word <= 32'd0; word_stb <= 1'b0;
        end else begin
            word_stb <= 1'b0;
            if (vn_stb) begin
                // x^32 + x^22 + x^2 + x + 1, feed the entropy bit into the input
                lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0] ^ vn_bit};
                if (bitcnt == 6'd31) begin
                    bitcnt <= 6'd0;
                    word <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0] ^ vn_bit};
                    word_stb <= 1'b1;
                end else begin
                    bitcnt <= bitcnt + 6'd1;
                end
            end
        end
    end

endmodule
