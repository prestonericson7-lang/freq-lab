`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// correlator_top.v -- PZ7020-StarLite two-channel cross-correlator
//
// Two XADC channels sampled synchronously, cross-correlated in real time.
// Measures the coherence between two signals: two ELF antennas at different
// locations (SRI's two-station experiment), two sensors on a body, or a
// drive signal vs. a pickup signal.
//
//   XADC ch A (VAUX1) ----+
//                         |---> correlator core ---> results via UART
//   XADC ch B (VAUX9) ----+
//                                                    +--- GPS PPS input
//
// The core computes per window (configurable N samples):
//   - Auto-correlation R_aa(0) = sum(a^2) / N     (channel A power)
//   - Auto-correlation R_bb(0) = sum(b^2) / N     (channel B power)
//   - Cross-correlation R_ab(lag) for lag = -MAX_LAG .. +MAX_LAG
//   - Cross power = sum(a*b) / N at lag 0
//
// Pins on JM1:
//   JM1-6   E17  ADC A+   0..1.0 V (VAUX1P)
//   JM1-8   D18  ADC A-   -> sensor A ground (VAUX1N)
//   JM1-9   E18  ADC B+   0..1.0 V (VAUX9P)
//   JM1-11  E19  ADC B-   -> sensor B ground (VAUX9N)
//   JM1-14  B19  UART TX
//   JM1-16  A20  UART RX
//   JM1-18  C20  GPS PPS (optional)
// (E18/E19 = AD9P/AD9N per the PZ-StarLite schematic and connector pin sheet.)
//-----------------------------------------------------------------------------
module correlator_top #(
    parameter integer CLK_HZ           = 50_000_000,
    parameter integer BAUD             = 115_200,
    parameter integer FAN_DUTY_DEFAULT = 60,
    parameter integer MAX_LAG          = 128
)(
    input  wire clk_50m,
    output wire led_hb,
    output wire led_act,
    input  wire key_n,

    output wire fan_pwm_o,
    input  wire fan_tach,

    input  wire vauxp1,           // channel A +
    input  wire vauxn1,           // channel A -
    input  wire vauxp9,           // channel B +
    input  wire vauxn9,           // channel B -

    input  wire gps_pps,          // GPS 1PPS (optional)
    output wire uart_tx,
    input  wire uart_rx
);
    wire clk = clk_50m;

    // ------------------------------------------------ reset
    localparam integer RST_TICKS = CLK_HZ / 1000;
    reg [$clog2(RST_TICKS):0] rst_cnt = 0;
    wire rst_n = rst_cnt[$clog2(RST_TICKS)];
    always @(posedge clk) if (!rst_n) rst_cnt <= rst_cnt + 1;

    // ------------------------------------------------ heartbeat
    reg [$clog2(CLK_HZ/2):0] hb_cnt = 0;
    reg hb = 0;
    always @(posedge clk) begin
        if (!rst_n) begin hb_cnt <= 0; hb <= 0; end
        else if (hb_cnt == CLK_HZ/2 - 1) begin hb_cnt <= 0; hb <= ~hb; end
        else hb_cnt <= hb_cnt + 1;
    end
    assign led_hb = hb;

    // ------------------------------------------------ fan
    wire [15:0] fan_rpm;
    reg  [7:0]  fan_duty = FAN_DUTY_DEFAULT;
    fan_pwm #(.CLK_HZ(CLK_HZ)) u_fan (
        .clk(clk), .rst_n(rst_n),
        .duty_pct((!key_n) ? 8'd100 : fan_duty),
        .fan_pwm(fan_pwm_o), .fan_tach(fan_tach), .rpm(fan_rpm));

    // ------------------------------------------------ dual-channel XADC
    wire [11:0] adc_a, adc_b;
    wire        stb_a,  stb_b;

    xadc_dual u_xadc (
        .clk(clk), .rst(~rst_n),
        .vauxp1(vauxp1), .vauxn1(vauxn1),
        .vauxp9(vauxp9), .vauxn9(vauxn9),
        .data_a(adc_a), .stb_a(stb_a),
        .data_b(adc_b), .stb_b(stb_b));

    // ------------------------------------------------ GPS PPS
    reg [2:0] pps_sync = 3'b0;
    always @(posedge clk) pps_sync <= {pps_sync[1:0], gps_pps};
    wire pps_rise = pps_sync[2:1] == 2'b01;

    reg [31:0] pps_counter = 0;       // sample counter since last PPS
    reg [31:0] pps_last    = 0;       // counter value at last PPS
    always @(posedge clk) begin
        if (!rst_n) begin
            pps_counter <= 0; pps_last <= 0;
        end else begin
            if (stb_a) pps_counter <= pps_counter + 1;
            if (pps_rise) begin
                pps_last    <= pps_counter;
                pps_counter <= 0;
            end
        end
    end

    // ------------------------------------------------ correlator core
    wire                  core_busy;
    wire                  core_done;
    wire signed [63:0]    raa;            // sum(a^2)
    wire signed [63:0]    rbb;            // sum(b^2)
    wire signed [63:0]    rab;            // sum(a*b) at lag 0
    wire [31:0]           core_n;
    wire signed [63:0]    sum_a, sum_b;   // sum(a), sum(b) for DC removal

    // cross-correlation at requested lag
    wire signed [63:0]    xcorr_val;
    wire                  xcorr_valid;

    // control signals from register file
    wire                  core_start;
    wire [23:0]           win_samples;
    wire signed [15:0]    query_lag;
    wire                  query_go;

    correlator_core #(.MAX_LAG(MAX_LAG)) u_core (
        .clk(clk), .rst_n(rst_n),
        .adc_a(adc_a), .adc_b(adc_b), .stb(stb_a),
        .start(core_start), .win_samples(win_samples),
        .busy(core_busy), .done(core_done),
        .raa(raa), .rbb(rbb), .rab(rab),
        .sum_a(sum_a), .sum_b(sum_b), .n_out(core_n),
        .query_lag(query_lag), .query_go(query_go),
        .xcorr_val(xcorr_val), .xcorr_valid(xcorr_valid));

    assign led_act = core_busy;

    // ------------------------------------------------ register file + UART
    cmd_regs_corr #(.CLKS_PER_BIT(CLK_HZ / BAUD)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .rx(uart_rx), .tx(uart_tx),
        .ctrl_start(core_start),
        .win_samples(win_samples),
        .query_lag(query_lag), .query_go(query_go),
        .busy(core_busy), .done(core_done),
        .raa(raa), .rbb(rbb), .rab(rab),
        .sum_a(sum_a), .sum_b(sum_b), .n_out(core_n),
        .xcorr_val(xcorr_val), .xcorr_valid(xcorr_valid),
        .adc_a(adc_a), .adc_b(adc_b),
        .pps_last(pps_last), .fan_rpm(fan_rpm));

endmodule
