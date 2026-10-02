`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// tb_lockin.v -- end-to-end simulation of lockin_top
//
// A behavioural "bench" is attached to the pins:
//   dac_out -> RC low-pass (1 kOhm, 10 nF) -> resonator (f0 = 20 kHz, Q = 25)
//           -> scaled and offset into 0..1 V -> the ADC model
// The test then talks to the design over its UART exactly as a host would:
// it measures the resonator at five frequencies, compares amplitude and phase
// lag with the analytic answer, and finally lets the tracker find the
// resonance starting 1 kHz low. (20 kHz keeps the simulation short; the logic
// is the same at 2 Hz.)
//
// Run:  iverilog -g2005 -DSIMULATION -o tb sim/tb_lockin.v hdl/*.v && vvp tb
//-----------------------------------------------------------------------------
module tb_lockin;
    localparam real    CLK_HZ   = 50.0e6;
    localparam integer BIT_NS   = 200;            // 5 Mbaud in simulation
    localparam real    PI       = 3.14159265358979;

    localparam real    F0       = 20000.0;
    localparam real    QF       = 25.0;
    localparam real    RC       = 10.0e-6;
    localparam real    SENSE_G  = 0.1;            // volts at the ADC per volt of resonator output
    localparam integer AMP_REG  = 5898;           // 10 % of maximum drive

    reg  clk = 1'b0;
    always #10 clk = ~clk;

    reg  uart_rxd = 1'b1;
    wire uart_txd, dac_out, sync_out, led_hb, led_act, fan_pwm;

    lockin_top #(.CLK_HZ(50_000_000), .BAUD(5_000_000)) dut (
        .clk_50m  (clk),
        .led_hb   (led_hb),
        .led_act  (led_act),
        .key_n    (1'b1),
        .fan_pwm  (fan_pwm),
        .fan_tach (1'b1),
        .vauxp1   (1'b0),
        .vauxn1   (1'b0),
        .dac_out  (dac_out),
        .sync_out (sync_out),
        .uart_tx  (uart_txd),
        .uart_rx  (uart_rxd)
    );

    // ------------------------------------------------------------ the bench
    real v_rc = 1.65, x = 0.0, v = 0.0, u, v_adc;
    real dt   = 20.0e-9;
    real w0;
    integer code;
    initial w0 = 2.0 * PI * F0;

    always @(posedge clk) begin
        v_rc = v_rc + ((dac_out ? 3.3 : 0.0) - v_rc) * (dt / RC);
        u    = v_rc - 1.65;
        v    = v + dt * (w0 * w0 * (u - x) - (w0 / QF) * v);
        x    = x + dt * v;
        v_adc = 0.5 + SENSE_G * x;
        if (v_adc < 0.0) v_adc = 0.0;
        if (v_adc > 0.99976) v_adc = 0.99976;
        code = $rtoi(v_adc * 4096.0);
        dut.u_adc.sim_code = code[11:0];
    end

    // ------------------------------------------------------------ UART host
    task send_byte(input [7:0] b);
        integer i;
        begin
            uart_rxd = 1'b0;               #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin uart_rxd = b[i]; #(BIT_NS); end
            uart_rxd = 1'b1;               #(BIT_NS);
        end
    endtask

    reg [7:0] rxbuf [0:15];
    integer   rxn = 0;
    always begin : rxproc
        reg [7:0] b;
        integer i;
        @(negedge uart_txd);
        #(BIT_NS / 2);
        for (i = 0; i < 8; i = i + 1) begin #(BIT_NS); b[i] = uart_txd; end
        #(BIT_NS / 2);
        rxbuf[rxn] = b;
        rxn = rxn + 1;
    end

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 10) ? (8'h30 + n) : (8'h37 + n);
    endfunction

    function [3:0] hexv(input [7:0] c);
        hexv = (c <= "9") ? c[3:0] : (c[3:0] + 4'd9);
    endfunction

    task wr(input [7:0] a, input [31:0] d);
        integer i;
        begin
            rxn = 0;
            send_byte("W");
            send_byte(hexc(a[7:4])); send_byte(hexc(a[3:0]));
            send_byte(" ");
            for (i = 7; i >= 0; i = i - 1) send_byte(hexc(d[4*i +: 4]));
            send_byte(8'h0A);
            wait (rxn == 2);
            if (rxbuf[0] != "K") begin $display("FAIL: write %02h not acknowledged (%c)", a, rxbuf[0]); errors = errors + 1; end
        end
    endtask

    task rd(input [7:0] a, output [31:0] d);
        integer i;
        begin
            rxn = 0;
            send_byte("R");
            send_byte(hexc(a[7:4])); send_byte(hexc(a[3:0]));
            send_byte(8'h0A);
            wait (rxn == 9);
            for (i = 0; i < 8; i = i + 1) d[4*(7-i) +: 4] = hexv(rxbuf[i]);
        end
    endtask

    task set_freq(input real f);
        reg [47:0] w;
        real wr_;
        begin
            wr_ = f * 281474976710656.0 / CLK_HZ;          // 2^48 / 50e6
            w   = wr_;
            wr(8'h02, w[31:0]);
            wr(8'h03, {16'd0, w[47:32]});
        end
    endtask

    // one triggered window; returns amplitude in ADC counts and lag in degrees
    integer errors = 0;
    task measure(output real amp_counts, output real lag_deg);
        reg [31:0] st0, st, n, ilo, ihi, qlo, qhi;
        reg signed [63:0] ri, rq;
        real fi, fq;
        begin
            rd(8'h08, st0);
            wr(8'h07, 32'd1);
            st = st0;
            while (st[23:8] == st0[23:8]) begin
                #100000;
                rd(8'h08, st);
            end
            rd(8'h09, n);
            rd(8'h0A, ilo); rd(8'h0B, ihi); rd(8'h0C, qlo); rd(8'h0D, qhi);
            ri = {ihi, ilo};
            rq = {qhi, qlo};
            fi = ri;                                  // 64-bit signed -> real
            fq = rq;
            amp_counts = 2.0 * $sqrt(fi * fi + fq * fq) / ($itor(n) * 32700.0);
            lag_deg    = $atan2(-fq, fi) * 180.0 / PI;
        end
    endtask

    real test_f [0:4];
    real a_meas, l_meas, a_exp, l_exp, r, hr, hi_, drive_v, rc_g, rc_l, dl;
    real lag_at_res;
    reg [31:0] tmp, flo, fhi;
    reg [47:0] fw;
    real f_track;
    integer k;

    initial begin
        test_f[0] = 16000.0; test_f[1] = 19000.0; test_f[2] = 20000.0;
        test_f[3] = 21000.0; test_f[4] = 25000.0;

        #2000000;                                    // past the 1 ms power-on reset
        rd(8'h00, tmp);
        if (tmp !== 32'h4C4B0001) begin $display("FAIL: ID = %08h", tmp); errors = errors + 1; end
        else $display("ID ok: %08h", tmp);

        wr(8'h04, AMP_REG);
        wr(8'h05, 32'd40);                           // 40 cycles per window
        wr(8'h01, 32'h11);                           // DAC on, sync on

        drive_v = 3.3 * ($itor(AMP_REG) * 32700.0 / 65536.0) / 65536.0;
        lag_at_res = 0.0;
        for (k = 0; k < 5; k = k + 1) begin
            set_freq(test_f[k]);
            #2500000;                                // 2.5 ms: six time constants of the resonator
            measure(a_meas, l_meas);
            r     = test_f[k] / F0;
            hr    = 1.0 - r * r;
            hi_   = r / QF;
            rc_g  = 1.0 / $sqrt(1.0 + (2.0 * PI * test_f[k] * RC) * (2.0 * PI * test_f[k] * RC));
            rc_l  = $atan(2.0 * PI * test_f[k] * RC) * 180.0 / PI;
            a_exp = drive_v * rc_g / $sqrt(hr * hr + hi_ * hi_) * SENSE_G * 4096.0;
            l_exp = $atan2(hi_, hr) * 180.0 / PI + rc_l;
            dl    = l_meas - l_exp;
            if (dl > 180.0) dl = dl - 360.0;
            if (dl < -180.0) dl = dl + 360.0;
            $display("f=%6.1f Hz  amp %8.2f counts (expected %8.2f)  lag %7.2f deg (expected %7.2f, error %+5.2f)",
                     test_f[k], a_meas, a_exp, l_meas, l_exp, dl);
            if ((a_meas < 0.97 * a_exp) || (a_meas > 1.03 * a_exp)) begin
                $display("FAIL: amplitude off by more than 3 %%"); errors = errors + 1;
            end
            if ((dl > 2.0) || (dl < -2.0)) begin
                $display("FAIL: lag off by more than 2 degrees"); errors = errors + 1;
            end
            if (k == 2) lag_at_res = l_meas;
        end

        // ---- resonance tracking: start 1 kHz low, 100 Hz per window
        set_freq(19000.0);
        wr(8'h05, 32'd10);                                          // 10-cycle windows
        wr(8'h13, $rtoi(32767.0 * $cos(lag_at_res * PI / 180.0)) & 32'hFFFF);
        wr(8'h14, $rtoi(32767.0 * $sin(lag_at_res * PI / 180.0)) & 32'hFFFF);
        wr(8'h12, $rtoi(100.0 * 281474976710656.0 / CLK_HZ));       // 100 Hz step
        wr(8'h01, 32'h17);                                          // DAC, continuous, track, sync
        #20000000;                                                  // 20 ms = 40 windows
        rd(8'h08, tmp);
        rd(8'h15, flo); rd(8'h16, fhi);
        fw = {fhi[15:0], flo};
        f_track = fw;                                               // 48-bit -> real
        f_track = f_track * CLK_HZ / 281474976710656.0;
        $display("tracker: started at 19000.0 Hz, now at %7.1f Hz (resonance %7.1f Hz), windows %0d",
                 f_track, F0, tmp[23:8]);
        if ((f_track < F0 - 250.0) || (f_track > F0 + 250.0)) begin
            $display("FAIL: tracker did not settle within 250 Hz of resonance"); errors = errors + 1;
        end

        if (errors == 0) $display("ALL TESTS PASSED");
        else             $display("%0d TEST(S) FAILED", errors);
        $finish;
    end

    initial begin
        #400000000;
        $display("FAIL: timeout");
        $finish;
    end
endmodule
