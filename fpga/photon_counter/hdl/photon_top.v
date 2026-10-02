`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// photon_top.v -- PZ7020-StarLite coincidence photon counter
//
// Two digital inputs from SiPM or APD pulse discriminators.  Each photon
// detection is a fast digital edge.  The FPGA timestamps every edge, counts
// singles on each channel, detects coincidences within a programmable time
// window, and computes the accidental rate so true coincidences can be
// extracted.
//
// This is the rig for the biophoton experiment (Gurwitsch 1920s, SRI 1987):
//   - Two photon detectors in coincidence reject dark counts and noise
//   - If a living tissue emits correlated photon pairs, the coincidence
//     rate exceeds the accidental rate
//   - The FPGA timestamps to 20 ns (one clock at 50 MHz); a carry-chain
//     TDC upgrade can reach ~200 ps if needed
//
// Pins on JM1:
//   JM1-6   E17  DET_A    LVCMOS33 input from detector A discriminator
//   JM1-8   D18  DET_B    LVCMOS33 input from detector B discriminator
//   JM1-10  F16  GATE     LVCMOS33 input, active-high counting gate (tie high
//                          if not used, or drive from a Teensy to control runs)
//   JM1-14  B19  UART TX
//   JM1-16  A20  UART RX
//
// The discriminator output must be a clean LVCMOS33 pulse: >20 ns wide,
// one pulse per photon, 0 V low / 3.3 V high.  Most SiPM breakout boards
// have a comparator output that meets this.  If you have a raw SiPM or APD
// signal, condition it with a fast comparator (LM311, LT1711 or similar)
// before connecting to the FPGA.
//
// DOCUMENTS
//   SRI photon production final report, CIA-RDP96-00789R002200190001-7
//   Gurwitsch mitogenetic radiation (1920s)
//   Chinese Journal of Somatic Science, CIA-RDP96-00792R000300040002-9
//
// WARNING
//   Do NOT expose the FPGA pins to voltages above 3.6 V or below -0.3 V.
//   SiPM bias supplies are typically 25-70 V and WILL destroy the FPGA if
//   connected directly.  The discriminator/comparator provides isolation.
//-----------------------------------------------------------------------------
module photon_top #(
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

    input  wire det_a,           // detector A pulse
    input  wire det_b,           // detector B pulse
    input  wire gate,            // counting gate (high = count)

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

    // ------------------------------------------------ input synchronizers
    reg [2:0] sync_a = 3'b0, sync_b = 3'b0, sync_g = 3'b0;
    always @(posedge clk) begin
        sync_a <= {sync_a[1:0], det_a};
        sync_b <= {sync_b[1:0], det_b};
        sync_g <= {sync_g[1:0], gate};
    end
    wire edge_a   = sync_a[2:1] == 2'b01;
    wire edge_b   = sync_b[2:1] == 2'b01;
    wire gate_on  = sync_g[2];

    // ------------------------------------------------ coincidence detector
    wire [47:0] singles_a, singles_b;
    wire [47:0] coincidences;
    wire [31:0] elapsed_clk;
    wire        coinc_busy;

    wire        coinc_start, coinc_stop;
    wire [15:0] coinc_window;             // coincidence window in clock ticks
    wire        coinc_done;

    coincidence u_coinc (
        .clk(clk), .rst_n(rst_n),
        .edge_a(edge_a & gate_on),
        .edge_b(edge_b & gate_on),
        .start(coinc_start), .stop(coinc_stop),
        .window(coinc_window),
        .busy(coinc_busy), .done(coinc_done),
        .singles_a(singles_a), .singles_b(singles_b),
        .coincidences(coincidences),
        .elapsed_clk(elapsed_clk));

    assign led_act = coinc_busy;

    // ------------------------------------------------ register file + UART
    cmd_regs_photon #(.CLKS_PER_BIT(CLK_HZ / BAUD)) u_regs (
        .clk(clk), .rst_n(rst_n),
        .rx(uart_rx), .tx(uart_tx),
        .coinc_start(coinc_start), .coinc_stop(coinc_stop),
        .coinc_window(coinc_window),
        .busy(coinc_busy), .done(coinc_done),
        .singles_a(singles_a), .singles_b(singles_b),
        .coincidences(coincidences),
        .elapsed_clk(elapsed_clk),
        .fan_rpm(fan_rpm));

endmodule
