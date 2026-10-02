`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// trng_top.v -- PZ7020-StarLite ring-oscillator TRNG with SP 800-90B health
// tests (list item #73). Noise source -> RCT + APT health tests -> von Neumann
// debiaser -> LFSR conditioning -> 32-bit words -> FIFO -> UART. On-chip
// temperature and VCCINT are logged alongside (SRI logged the diode temp).
//
// Pins (JM1, same as the other freq-lab designs; this design uses no analog pins):
//   JM1-14 B19 UART TX  -> adapter RX      JM1-16 A20 UART RX <- adapter TX
//   JM1-5  H16 fan PWM   JM1-7 H17 fan tach   LED R19 heartbeat, V13 = health OK
//   KEY1 G14 held = fan 100 %
// Nothing in this design transmits on radio; it only generates random numbers.
//-----------------------------------------------------------------------------
module trng_top #(
    parameter integer CLK_HZ = 50_000_000,
    parameter integer BAUD   = 115_200,
    parameter integer FAN_DUTY_DEFAULT = 60
)(
    input  wire clk_50m,
    output wire led_hb,
    output wire led_act,
    input  wire key_n,
    output wire fan_pwm_o,
    input  wire fan_tach,
    output wire uart_tx,
    input  wire uart_rx
);
    wire clk = clk_50m;

    // reset: 2^16 clocks after configuration
    reg [16:0] rst_cnt = 0;
    wire rst_n = rst_cnt[16];
    always @(posedge clk) if (!rst_n) rst_cnt <= rst_cnt + 1'b1;

    // heartbeat
    reg [$clog2(CLK_HZ/2):0] hb_cnt = 0; reg hb = 0;
    always @(posedge clk) begin
        if (!rst_n) begin hb_cnt<=0; hb<=1'b0; end
        else if (hb_cnt==CLK_HZ/2-1) begin hb_cnt<=0; hb<=~hb; end
        else hb_cnt<=hb_cnt+1'b1;
    end
    assign led_hb = hb;

    // register-file outputs
    wire        run, clear_alarms;
    wire [5:0]  rct_cutoff;
    wire [15:0] apt_window, apt_cutoff;
    wire [7:0]  fan_duty;

    // fan
    wire [15:0] fan_rpm;
    fan_pwm #(.CLK_HZ(CLK_HZ)) u_fan (.clk(clk), .rst_n(rst_n),
        .duty_pct((!key_n)?8'd100:fan_duty), .fan_pwm(fan_pwm_o), .fan_tach(fan_tach), .rpm(fan_rpm));

    // XADC temperature / vccint
    wire [11:0] temp_code, vccint_code;
    xadc_temp u_xadc (.clk(clk), .rst(~rst_n), .temp_code(temp_code), .vccint_code(vccint_code));

    // --------- noise source ---------
    // sample one raw bit roughly every 16 clocks (decimation inside entropy_src)
    reg [3:0] rate; reg raw_tick;
    always @(posedge clk) begin
        if (!rst_n) begin rate<=4'd0; raw_tick<=1'b0; end
        else begin rate<=rate+4'd1; raw_tick<=(rate==4'd0); end
    end
    wire raw_bit, raw_stb;
    entropy_src #(.NOSC(16), .RLEN(3), .DECIM(8)) u_src (
        .clk(clk), .rst_n(rst_n), .en(run),
        .sim_bit(1'b0), .sim_stb(raw_tick),      // in hardware sim_bit is ignored; raw_tick paces the sim path
        .raw_bit(raw_bit), .raw_stb(raw_stb));

    // --------- pipeline ---------
    wire [31:0] word; wire word_stb;
    wire rct_fail, apt_fail;
    wire [31:0] raw_count, vn_count, rct_fail_count, apt_fail_count;
    trng_core u_core (
        .clk(clk), .rst_n(rst_n),
        .raw_bit(raw_bit), .raw_stb(raw_stb),
        .clear_alarms(clear_alarms),
        .rct_cutoff(rct_cutoff), .apt_window(apt_window), .apt_cutoff(apt_cutoff),
        .word(word), .word_stb(word_stb),
        .rct_fail(rct_fail), .apt_fail(apt_fail),
        .raw_count(raw_count), .vn_count(vn_count),
        .rct_fail_count(rct_fail_count), .apt_fail_count(apt_fail_count));

    // total words produced
    reg [31:0] word_count;
    always @(posedge clk) begin
        if (!rst_n) word_count<=32'd0;
        else if (word_stb) word_count<=word_count+32'd1;
    end

    // health LED: solid when the source is healthy, off while a health alarm is latched
    assign led_act = run & ~rct_fail & ~apt_fail;

    // --------- register file + UART ---------
    cmd_regs_trng #(.CLKS_PER_BIT(CLK_HZ/BAUD), .FAN_DUTY_DEFAULT(FAN_DUTY_DEFAULT)) u_regs (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx), .tx(uart_tx),
        .run(run), .clear_alarms(clear_alarms),
        .rct_cutoff(rct_cutoff), .apt_window(apt_window), .apt_cutoff(apt_cutoff), .fan_duty(fan_duty),
        .word_in(word), .word_stb(word_stb),
        .rct_fail(rct_fail), .apt_fail(apt_fail),
        .raw_count(raw_count), .vn_count(vn_count),
        .rct_fail_count(rct_fail_count), .apt_fail_count(apt_fail_count), .word_count(word_count),
        .temp_code(temp_code), .vccint_code(vccint_code), .fan_rpm(fan_rpm));

endmodule
