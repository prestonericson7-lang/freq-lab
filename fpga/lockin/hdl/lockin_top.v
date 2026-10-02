`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// lockin_top.v -- PZ7020-StarLite resonance analyzer (PL only, no PS needed)
//
// A sine generator, the Zynq's own 12-bit ADC and a digital lock-in on one
// clock, controlled from a PC or a Teensy over a 3.3 V UART:
//
//   48-bit DDS -> CORDIC sin/cos -> sigma-delta DAC pin -> (RC filter) -> exciter
//                       |                                                   |
//                       +-> lock-in multiply/accumulate <- XADC <- sensor <-+
//
// The drive and the two references are the same numbers, so the measurement is
// phase-locked to the drive by construction. Resonance tracking steps the drive
// frequency each window to hold a chosen phase lag.
//
// Pins (PACKAGE_PIN balls, see constraints/lockin_jm1.xdc and the board repo's
// docs/pinout.md). Everything new is on JM1's even-numbered row:
//
//   clk_50m   U18            50 MHz fabric clock
//   led_hb    R19   LED1     ~1 Hz heartbeat
//   led_act   V13   LED2     toggles on every finished lock-in window
//   key_n     G14   KEY1     hold: fan to 100 %
//   fan_pwm   H16   JM1-5    unchanged from fan_top.v, so the fan keeps running
//   fan_tach  H17   JM1-7
//   vauxp1    E17   JM1-6    ADC input, 0..1.0 V ONLY
//   vauxn1    D18   JM1-8    ADC return: wire to the sensor's ground
//   dac_out   F16   JM1-10   sigma-delta drive, needs an RC low-pass
//   sync_out  F17   JM1-12   square wave at the drive frequency
//   uart_tx   B19   JM1-14   -> RX of a 3.3 V USB-serial adapter (or Teensy RX)
//   uart_rx   A20   JM1-16   <- TX of the adapter (or Teensy TX)
//   (JM1-2 is 3.3 V, JM1-4 is GND)
//-----------------------------------------------------------------------------
module lockin_top #(
    parameter integer CLK_HZ           = 50_000_000,
    parameter integer BAUD             = 115_200,
    parameter integer FAN_DUTY_DEFAULT = 60
)(
    input  wire clk_50m,
    output wire led_hb,
    output wire led_act,
    input  wire key_n,

    output wire fan_pwm,
    input  wire fan_tach,

    input  wire vauxp1,
    input  wire vauxn1,
    output wire dac_out,
    output wire sync_out,
    output wire uart_tx,
    input  wire uart_rx
);
    localparam integer CORDIC_LATENCY = 18;

    wire clk = clk_50m;

    // ------------------------------------------------------------------
    // Power-on reset: about 1 ms after configuration
    // ------------------------------------------------------------------
    localparam integer RST_TICKS = CLK_HZ / 1000;
    reg [$clog2(RST_TICKS+1)-1:0] rst_cnt = 0;
    reg                           rst_n   = 1'b0;

    always @(posedge clk) begin
        if (rst_cnt == RST_TICKS) rst_n <= 1'b1;
        else                      rst_cnt <= rst_cnt + 1'b1;
    end

    // ------------------------------------------------------------------
    // Heartbeat
    // ------------------------------------------------------------------
    reg [31:0] hb_cnt = 0;
    reg        hb     = 0;

    always @(posedge clk) begin
        if (hb_cnt == (CLK_HZ/2) - 1) begin
            hb_cnt <= 32'd0;
            hb     <= ~hb;
        end else begin
            hb_cnt <= hb_cnt + 32'd1;
        end
    end
    assign led_hb = hb;

    // ------------------------------------------------------------------
    // Fan: same behaviour as the repo's fan_top.v
    // ------------------------------------------------------------------
    wire [7:0]  fan_duty = key_n ? FAN_DUTY_DEFAULT[7:0] : 8'd100;
    wire [15:0] fan_rpm;

    fan_pwm #(
        .CLK_HZ              (CLK_HZ),
        .PWM_HZ              (25_000),
        .TACH_PULSES_PER_REV (2)
    ) u_fan (
        .clk      (clk),
        .rst_n    (rst_n),
        .duty_pct (fan_duty),
        .fan_pwm  (fan_pwm),
        .fan_tach (fan_tach),
        .rpm      (fan_rpm)
    );

    // ------------------------------------------------------------------
    // Registers and UART command port
    // ------------------------------------------------------------------
    wire [4:0]         ctrl;
    wire [47:0]        ftw_set;
    wire               ftw_load;
    wire [15:0]        amp;
    wire [23:0]        cycles;
    wire [4:0]         dc_shift;
    wire               start;
    wire [31:0]        track_step;
    wire signed [15:0] track_cos;
    wire signed [15:0] track_sin;

    wire               busy;
    wire               done;
    wire signed [63:0] res_i;
    wire signed [63:0] res_q;
    wire [31:0]        res_n;
    wire [47:0]        res_sumx;
    wire [11:0]        adc_last;
    wire [11:0]        dc_est;
    reg                adc_seen;
    reg  [47:0]        ftw_now;

    wire dac_en     = ctrl[0];
    wire continuous = ctrl[1];
    wire track_en   = ctrl[2];
    wire track_inv  = ctrl[3];
    wire sync_en    = ctrl[4];

    cmd_regs #(.CLKS_PER_BIT(CLK_HZ / BAUD)) u_regs (
        .clk        (clk),
        .rst_n      (rst_n),
        .rx         (uart_rx),
        .tx         (uart_tx),
        .ctrl       (ctrl),
        .ftw_set    (ftw_set),
        .ftw_load   (ftw_load),
        .amp        (amp),
        .cycles     (cycles),
        .dc_shift   (dc_shift),
        .start      (start),
        .track_step (track_step),
        .track_cos  (track_cos),
        .track_sin  (track_sin),
        .busy       (busy),
        .adc_seen   (adc_seen),
        .done       (done),
        .res_i      (res_i),
        .res_q      (res_q),
        .res_n      (res_n),
        .res_sumx   (res_sumx),
        .adc_last   (adc_last),
        .dc_est     (dc_est),
        .ftw_now    (ftw_now)
    );

    // ------------------------------------------------------------------
    // DDS: 48-bit phase accumulator, 50e6 / 2^48 = 0.18 micro-Hz per step
    // ------------------------------------------------------------------
    reg  [47:0] phase;
    reg         wrap_raw;
    wire [48:0] phase_next = {1'b0, phase} + {1'b0, ftw_now};

    always @(posedge clk) begin
        if (!rst_n) begin
            phase    <= 48'd0;
            wrap_raw <= 1'b0;
        end else begin
            phase    <= phase_next[47:0];
            wrap_raw <= phase_next[48];
        end
    end

    wire signed [15:0] ref_sin;
    wire signed [15:0] ref_cos;

    cordic_sincos u_cordic (
        .clk   (clk),
        .phase (phase[47:28]),
        .sin_o (ref_sin),
        .cos_o (ref_cos)
    );

    // Delay the cycle marker and the half-cycle bit by the CORDIC latency so
    // they line up with the sine that comes out of it.
    reg [CORDIC_LATENCY-1:0] wrap_dl;
    reg [CORDIC_LATENCY-1:0] msb_dl;

    always @(posedge clk) begin
        if (!rst_n) begin
            wrap_dl <= {CORDIC_LATENCY{1'b0}};
            msb_dl  <= {CORDIC_LATENCY{1'b0}};
        end else begin
            wrap_dl <= {wrap_dl[CORDIC_LATENCY-2:0], wrap_raw};
            msb_dl  <= {msb_dl[CORDIC_LATENCY-2:0], phase[47]};
        end
    end

    wire wrap = wrap_dl[CORDIC_LATENCY-1];
    assign sync_out = sync_en & ~msb_dl[CORDIC_LATENCY-1];   // high during the first half cycle

    // ------------------------------------------------------------------
    // Drive output: sine * amplitude, offset to mid-scale, 1-bit DAC
    // ------------------------------------------------------------------
    wire signed [16:0] amp_s    = {1'b0, amp};
    wire signed [32:0] drv_prod = ref_sin * amp_s;
    reg  [15:0]        dac_val;

    always @(posedge clk) begin
        if (!rst_n)       dac_val <= 16'h8000;
        else if (dac_en)  dac_val <= 16'h8000 + drv_prod[31:16];
        else              dac_val <= 16'h8000;
    end

    sd_dac u_dac (
        .clk   (clk),
        .rst_n (rst_n),
        .din   (dac_val),
        .dout  (dac_out)
    );

    // ------------------------------------------------------------------
    // ADC
    // ------------------------------------------------------------------
    wire [11:0] adc_data;
    wire        adc_stb;

    xadc_vaux1 u_adc (
        .clk    (clk),
        .rst    (~rst_n),
        .vauxp1 (vauxp1),
        .vauxn1 (vauxn1),
        .data   (adc_data),
        .stb    (adc_stb)
    );

    always @(posedge clk) begin
        if (!rst_n)       adc_seen <= 1'b0;
        else if (adc_stb) adc_seen <= 1'b1;
    end

    // ------------------------------------------------------------------
    // Lock-in
    // ------------------------------------------------------------------
    lockin_core u_lockin (
        .clk        (clk),
        .rst_n      (rst_n),
        .adc_stb    (adc_stb),
        .adc_data   (adc_data),
        .ref_sin    (ref_sin),
        .ref_cos    (ref_cos),
        .wrap       (wrap),
        .start      (start),
        .continuous (continuous),
        .cycles     (cycles),
        .dc_shift   (dc_shift),
        .busy       (busy),
        .done       (done),
        .res_i      (res_i),
        .res_q      (res_q),
        .res_n      (res_n),
        .res_sumx   (res_sumx),
        .adc_last   (adc_last),
        .dc_est     (dc_est)
    );

    reg act = 1'b0;
    always @(posedge clk) if (done) act <= ~act;
    assign led_act = act;

    // ------------------------------------------------------------------
    // Resonance tracker
    //
    // For an input A*sin(phase - lag):  I ~ cos(lag),  Q ~ -sin(lag).
    // With the target lag given as (cos, sin), the quantity
    //     e = I*sin(target) + Q*cos(target)  =  -(NA/2) * sin(lag - target)
    // is negative when the response lags more than the target. Across a
    // resonance the lag grows with frequency, so "lags too much" means the
    // drive is above resonance: step the frequency down. Otherwise step up.
    // One step per window; the step size is set by the host.
    // ------------------------------------------------------------------
    reg [2:0]          trk_sr;
    reg signed [39:0]  trk_i, trk_q;
    reg signed [55:0]  trk_pi, trk_pq;
    wire signed [56:0] trk_e = {trk_pi[55], trk_pi} + {trk_pq[55], trk_pq};
    wire               step_down = trk_e[56] ^ track_inv;

    always @(posedge clk) begin
        if (!rst_n) begin
            ftw_now <= 48'd0;
            trk_sr  <= 3'b000;
            trk_i   <= 40'sd0;
            trk_q   <= 40'sd0;
            trk_pi  <= 56'sd0;
            trk_pq  <= 56'sd0;
        end else begin
            trk_sr <= {trk_sr[1:0], done};

            if (trk_sr[0]) begin                       // results are stable one clock after done
                trk_i <= res_i[63:24];
                trk_q <= res_q[63:24];
            end
            if (trk_sr[1]) begin
                trk_pi <= trk_i * track_sin;
                trk_pq <= trk_q * track_cos;
            end

            if (ftw_load) begin
                ftw_now <= ftw_set;
            end else if (trk_sr[2] && track_en && (trk_e != 57'sd0)) begin
                if (step_down) begin
                    if (ftw_now > {16'd0, track_step}) ftw_now <= ftw_now - {16'd0, track_step};
                end else begin
                    ftw_now <= ftw_now + {16'd0, track_step};
                end
            end
        end
    end

endmodule
