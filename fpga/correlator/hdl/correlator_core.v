`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// correlator_core.v -- real-time cross-correlation engine
//
// On START, accumulates a window of N sample pairs (a, b).  After the window
// closes, the results are available:
//   raa       = sum(a_dc^2)               (auto-correlation of A at lag 0)
//   rbb       = sum(b_dc^2)               (auto-correlation of B at lag 0)
//   rab       = sum(a_dc * b_dc)           (cross-correlation at lag 0)
//   sum_a/b   = sum(a), sum(b)             (for DC computation)
//
// DC is removed: a_dc = a - mean(a). Because the mean is not known until the
// window closes, we compute using the identity:
//   sum((a-ma)*(b-mb)) = sum(a*b) - N*ma*mb
//                      = sum(a*b) - sum(a)*sum(b)/N
// The host divides sum_a*sum_b/N and subtracts from rab.
//
// Cross-correlation at nonzero lags is done by query: set query_lag and pulse
// query_go.  The core keeps a circular buffer of the last MAX_LAG*2 samples
// and computes the requested lag from the buffer.  This avoids storing all
// lag bins simultaneously (saves block RAM).
//
// All arithmetic is 25-bit signed (12 unsigned ADC -> subtract 2048 -> 13-bit
// signed -> products are 26 bits -> accumulated into 64 bits).
//-----------------------------------------------------------------------------
module correlator_core #(
    parameter integer MAX_LAG = 128       // maximum +/- lag in samples
)(
    input  wire               clk,
    input  wire               rst_n,

    input  wire [11:0]        adc_a,
    input  wire [11:0]        adc_b,
    input  wire               stb,        // one pulse per sample pair

    // control
    input  wire               start,
    input  wire [23:0]        win_samples, // window length (1 .. 2^24)

    // status
    output reg                busy,
    output reg                done,        // one pulse when results ready

    // results (valid while !busy after done)
    output reg  signed [63:0] raa,
    output reg  signed [63:0] rbb,
    output reg  signed [63:0] rab,
    output reg  signed [63:0] sum_a,
    output reg  signed [63:0] sum_b,
    output reg  [31:0]        n_out,

    // lag query interface
    input  wire signed [15:0] query_lag,   // -MAX_LAG .. +MAX_LAG
    input  wire               query_go,    // pulse: compute xcorr at this lag
    output reg  signed [63:0] xcorr_val,   // result
    output reg                xcorr_valid  // one pulse when xcorr_val ready
);

    localparam integer BUF_SIZE = MAX_LAG * 2;
    localparam integer BW       = $clog2(BUF_SIZE);

    // DC-removed signed samples
    wire signed [12:0] sa = {1'b0, adc_a} - 13'sd2048;
    wire signed [12:0] sb = {1'b0, adc_b} - 13'sd2048;

    // circular buffer for lag queries
    reg signed [12:0] buf_a [0:BUF_SIZE-1];
    reg signed [12:0] buf_b [0:BUF_SIZE-1];
    reg [BW-1:0]      buf_wr = 0;

    // accumulators
    reg signed [63:0] acc_aa, acc_bb, acc_ab;
    reg signed [63:0] acc_sa, acc_sb;
    reg [23:0]        sample_cnt;
    reg [23:0]        win_len;

    // lag query state machine
    reg [2:0]         qstate;
    localparam Q_IDLE = 3'd0, Q_INIT = 3'd1, Q_ACC = 3'd2, Q_DONE = 3'd3;
    reg signed [15:0] q_lag;
    reg [BW-1:0]      q_rd_a, q_rd_b;
    reg [23:0]        q_cnt;
    reg signed [63:0] q_acc;
    reg [BW-1:0]      q_idx_a, q_idx_b;
    reg signed [12:0] q_va, q_vb;

    integer i;

    always @(posedge clk) begin
        if (!rst_n) begin
            busy       <= 1'b0;
            done       <= 1'b0;
            acc_aa     <= 64'sd0;
            acc_bb     <= 64'sd0;
            acc_ab     <= 64'sd0;
            acc_sa     <= 64'sd0;
            acc_sb     <= 64'sd0;
            sample_cnt <= 24'd0;
            win_len    <= 24'd1000;
            buf_wr     <= {BW{1'b0}};
            raa        <= 64'sd0;
            rbb        <= 64'sd0;
            rab        <= 64'sd0;
            sum_a      <= 64'sd0;
            sum_b      <= 64'sd0;
            n_out      <= 32'd0;
            qstate     <= Q_IDLE;
            xcorr_val  <= 64'sd0;
            xcorr_valid <= 1'b0;
        end else begin
            done        <= 1'b0;
            xcorr_valid <= 1'b0;

            // ---- start a new window
            if (start && !busy) begin
                busy       <= 1'b1;
                win_len    <= (win_samples == 24'd0) ? 24'd1000 : win_samples;
                acc_aa     <= 64'sd0;
                acc_bb     <= 64'sd0;
                acc_ab     <= 64'sd0;
                acc_sa     <= 64'sd0;
                acc_sb     <= 64'sd0;
                sample_cnt <= 24'd0;
            end

            // ---- accumulate on each sample strobe
            if (busy && stb) begin
                // store in circular buffer
                buf_a[buf_wr] <= sa;
                buf_b[buf_wr] <= sb;
                buf_wr        <= (buf_wr == BUF_SIZE - 1) ? {BW{1'b0}} : buf_wr + 1;

                // accumulate products (raw, DC removed by host)
                acc_aa <= acc_aa + sa * sa;
                acc_bb <= acc_bb + sb * sb;
                acc_ab <= acc_ab + sa * sb;
                acc_sa <= acc_sa + sa;
                acc_sb <= acc_sb + sb;

                sample_cnt <= sample_cnt + 24'd1;

                if (sample_cnt + 24'd1 == win_len) begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    raa   <= acc_aa + sa * sa;     // include this last sample
                    rbb   <= acc_bb + sb * sb;
                    rab   <= acc_ab + sa * sb;
                    sum_a <= acc_sa + sa;
                    sum_b <= acc_sb + sb;
                    n_out <= {8'd0, win_len};
                end
            end

            // ---- lag query state machine
            // Reads through the circular buffer and computes
            // sum(a[i] * b[i + lag]) for all stored samples.
            // This works on the last BUF_SIZE samples in the buffer.
            case (qstate)
                Q_IDLE: begin
                    if (query_go && !busy) begin
                        q_lag  <= query_lag;
                        q_acc  <= 64'sd0;
                        q_cnt  <= 24'd0;
                        qstate <= Q_ACC;
                    end
                end

                Q_ACC: begin
                    // read a[wr + cnt] and b[wr + cnt + lag] from circular buffer
                    // both indices wrap modulo BUF_SIZE
                    // We can compute one product per clock (DSP48 can do 25x18 in one cycle)
                    q_idx_a = buf_wr + q_cnt[BW-1:0];
                    if (q_lag >= 0)
                        q_idx_b = buf_wr + q_cnt[BW-1:0] + q_lag[BW-1:0];
                    else
                        q_idx_b = buf_wr + q_cnt[BW-1:0] - (-q_lag[BW-1:0]);
                    q_va = buf_a[q_idx_a];
                    q_vb = buf_b[q_idx_b];
                    q_acc <= q_acc + q_va * q_vb;
                    q_cnt <= q_cnt + 24'd1;
                    if (q_cnt + 24'd1 >= BUF_SIZE) begin
                        qstate <= Q_DONE;
                    end
                end

                Q_DONE: begin
                    xcorr_val  <= q_acc;
                    xcorr_valid <= 1'b1;
                    qstate     <= Q_IDLE;
                end

                default: qstate <= Q_IDLE;
            endcase
        end
    end

endmodule
