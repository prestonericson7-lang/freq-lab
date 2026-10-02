`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// lockin_core.v -- digital lock-in: measure the part of the ADC signal that is
//                  at the drive frequency, as in-phase and quadrature sums
//
// Every ADC sample has its slow average (DC) removed, is multiplied by the
// reference sine and cosine, and the products are summed over a whole number of
// reference cycles. For an input  x = A*sin(phase - lag):
//
//     I = sum(x * sin_ref) =  (N * A * 32700 / 2) * cos(lag)
//     Q = sum(x * cos_ref) = -(N * A * 32700 / 2) * sin(lag)
//
// so   A (in ADC counts) = 2 * sqrt(I^2 + Q^2) / (N * 32700)
//      lag               = atan2(-Q, I)
//
// A window starts on a reference-cycle boundary (`wrap`) and ends `cycles`
// boundaries later, which makes the sums blind to DC and to anything that is
// not at the drive frequency or its near neighbours.
//-----------------------------------------------------------------------------
module lockin_core (
    input  wire               clk,
    input  wire               rst_n,

    input  wire               adc_stb,      // one clock pulse per ADC sample
    input  wire [11:0]        adc_data,     // unsigned code

    input  wire signed [15:0] ref_sin,
    input  wire signed [15:0] ref_cos,
    input  wire               wrap,         // one clock pulse per reference cycle

    input  wire               start,        // pulse: measure one window
    input  wire               continuous,   // level: measure back to back
    input  wire [23:0]        cycles,       // reference cycles per window (0 acts as 1)
    input  wire [4:0]         dc_shift,     // DC tracker time constant = 2^dc_shift samples

    output wire               busy,
    output reg                done,         // one clock pulse when results are latched
    output reg  signed [63:0] res_i,
    output reg  signed [63:0] res_q,
    output reg  [31:0]        res_n,        // samples in the window
    output reg  [47:0]        res_sumx,     // sum of raw codes (mean = sumx / n)
    output reg  [11:0]        adc_last,
    output wire [11:0]        dc_est
);

    // ------------------------------------------------------------------
    // DC tracker: dc += (x - dc) / 2^dc_shift, 12.32 fixed point
    // ------------------------------------------------------------------
    reg  signed [44:0] dc_acc;                                   // sign + 12.32
    wire signed [44:0] x_fix  = {1'b0, adc_data, 32'd0};
    wire signed [44:0] dc_err = x_fix - dc_acc;
    assign dc_est = dc_acc[43:32];

    // ------------------------------------------------------------------
    // Sample pipeline: subtract DC -> multiply -> accumulate
    // ------------------------------------------------------------------
    reg signed [12:0] xs;                    // sample minus DC
    reg signed [15:0] s_r, c_r;
    reg [11:0]        raw_r;
    reg               v1, v2;
    reg signed [28:0] prod_i, prod_q;

    reg signed [63:0] acc_i, acc_q;
    reg [31:0]        n_cnt;
    reg [47:0]        sumx;

    reg               running;
    reg               armed;
    reg [23:0]        cyc_cnt;
    wire [23:0]       cycles_eff = (cycles == 24'd0) ? 24'd1 : cycles;

    assign busy = running | armed;

    always @(posedge clk) begin
        if (!rst_n) begin
            dc_acc   <= {1'b0, 12'd2048, 32'd0};
            adc_last <= 12'd0;
            xs       <= 13'sd0;
            s_r      <= 16'sd0;
            c_r      <= 16'sd0;
            raw_r    <= 12'd0;
            v1       <= 1'b0;
            v2       <= 1'b0;
            prod_i   <= 29'sd0;
            prod_q   <= 29'sd0;
            acc_i    <= 64'sd0;
            acc_q    <= 64'sd0;
            n_cnt    <= 32'd0;
            sumx     <= 48'd0;
            running  <= 1'b0;
            armed    <= 1'b0;
            cyc_cnt  <= 24'd0;
            done     <= 1'b0;
            res_i    <= 64'sd0;
            res_q    <= 64'sd0;
            res_n    <= 32'd0;
            res_sumx <= 48'd0;
        end else begin
            done <= 1'b0;

            // stage 1: capture the sample with the reference that is live now
            v1 <= adc_stb;
            if (adc_stb) begin
                adc_last <= adc_data;
                raw_r    <= adc_data;
                xs       <= $signed({1'b0, adc_data}) - $signed({1'b0, dc_acc[43:32]});
                s_r      <= ref_sin;
                c_r      <= ref_cos;
                dc_acc   <= dc_acc + (dc_err >>> dc_shift);
            end

            // stage 2: multiply
            v2 <= v1;
            if (v1) begin
                prod_i <= xs * s_r;
                prod_q <= xs * c_r;
            end

            // stage 3: accumulate while a window is open
            if (v2 && running) begin
                acc_i <= acc_i + prod_i;
                acc_q <= acc_q + prod_q;
                n_cnt <= n_cnt + 32'd1;
                sumx  <= sumx + {36'd0, raw_r};
            end

            // window control, on reference-cycle boundaries
            if (start) armed <= 1'b1;

            if (wrap) begin
                if (running) begin
                    if (cyc_cnt + 24'd1 >= cycles_eff) begin
                        res_i    <= acc_i;
                        res_q    <= acc_q;
                        res_n    <= n_cnt;
                        res_sumx <= sumx;
                        done     <= 1'b1;
                        acc_i    <= 64'sd0;
                        acc_q    <= 64'sd0;
                        n_cnt    <= 32'd0;
                        sumx     <= 48'd0;
                        cyc_cnt  <= 24'd0;
                        if (!continuous) running <= 1'b0;
                    end else begin
                        cyc_cnt <= cyc_cnt + 24'd1;
                    end
                end else if (armed || continuous) begin
                    running <= 1'b1;
                    armed   <= 1'b0;
                    acc_i   <= 64'sd0;
                    acc_q   <= 64'sd0;
                    n_cnt   <= 32'd0;
                    sumx    <= 48'd0;
                    cyc_cnt <= 24'd0;
                end
            end
        end
    end

endmodule
