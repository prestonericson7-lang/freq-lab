`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// tb_correlator.v -- testbench for the correlator design
//
// Uses a real uart_rx instance for receive (not manual # delays, which
// race with NBA updates in iverilog and garble data).
//
// Tests:
//   1. Reset and ID read
//   2. Live ADC reads (registers 0F, 10)
//   3. Set window length, start acquisition, wait for done
//   4. Read results (STATUS freezes snapshot, then read accumulators)
//   5. Lag query: set lag, pulse query_go, read result
//   6. Write error (bad nibble count) -> "E\n"
//   7. GPS PPS counter
//
// Run with:
//   iverilog -DSIMULATION -o tb_correlator \
//       sim/tb_correlator.v hdl/correlator_top.v hdl/correlator_core.v \
//       hdl/xadc_dual.v hdl/cmd_regs_corr.v hdl/uart.v hdl/fan_pwm.v
//   vvp tb_correlator
//-----------------------------------------------------------------------------
module tb_correlator;

    localparam integer CLK_HZ   = 50_000_000;
    localparam integer BAUD     = 115_200;
    localparam integer CPB      = CLK_HZ / BAUD;   // 434
    localparam integer BIT_NS   = CPB * 20;         // ~8680 ns per bit
    localparam integer BYTE_NS  = BIT_NS * 10;      // 10 bits per 8N1 frame

    reg  clk = 0;
    always #10 clk = ~clk;     // 50 MHz

    reg  rst_pulse = 1;
    wire led_hb, led_act;
    wire fan_pwm_o;
    reg  fan_tach = 1;
    reg  gps_pps  = 0;
    wire uart_tx;
    reg  uart_rx_pin = 1;      // idle high

    correlator_top #(
        .CLK_HZ          (CLK_HZ),
        .BAUD            (BAUD),
        .FAN_DUTY_DEFAULT(60),
        .MAX_LAG         (16)          // small for fast simulation
    ) dut (
        .clk_50m  (clk),
        .led_hb   (led_hb),
        .led_act  (led_act),
        .key_n    (1'b1),
        .fan_pwm_o(fan_pwm_o),
        .fan_tach (fan_tach),
        .vauxp1   (1'b0),
        .vauxn1   (1'b0),
        .vauxp9   (1'b0),
        .vauxn9   (1'b0),
        .gps_pps  (gps_pps),
        .uart_tx  (uart_tx),
        .uart_rx  (uart_rx_pin)
    );

    // ---- Testbench UART receiver (proven correct by loopback test) ------
    wire [7:0] tb_rx_data;
    wire       tb_rx_valid;
    reg        tb_rst_n = 0;

    uart_rx #(.CLKS_PER_BIT(CPB)) tb_rx_inst (
        .clk  (clk),
        .rst_n(tb_rst_n),
        .rx   (uart_tx),
        .data (tb_rx_data),
        .valid(tb_rx_valid)
    );

    // ---- UART send task (8N1, LSB first) --------------------------------
    task uart_send(input [7:0] byte_val);
        integer i;
        begin
            uart_rx_pin = 0;                       // start bit
            #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = byte_val[i];
                #(BIT_NS);
            end
            uart_rx_pin = 1;                       // stop bit
            #(BIT_NS);
        end
    endtask

    // ---- UART receive task (uses uart_rx instance) ----------------------
    reg [7:0] rx_byte;
    task uart_recv;
        begin
            @(posedge tb_rx_valid);
            @(posedge clk);   // let data settle one clock
            rx_byte = tb_rx_data;
        end
    endtask

    // ---- send a whole string --------------------------------------------
    task uart_send_str(input [255:0] str, input integer len);
        integer i;
        begin
            for (i = len - 1; i >= 0; i = i - 1) begin
                uart_send(str[i*8 +: 8]);
            end
        end
    endtask

    // ---- receive N chars, print them ------------------------------------
    task uart_recv_line(input integer expect_chars);
        integer i;
        begin
            for (i = 0; i < expect_chars; i = i + 1) begin
                uart_recv;
                $write("%c", rx_byte);
            end
            $write("\n");
        end
    endtask

    // ---- receive and verify against expected string ---------------------
    integer pass_count = 0;
    integer fail_count = 0;

    task recv_and_check(input [255:0] expected, input integer len, input [255:0] label);
        integer i;
        reg [7:0] exp_ch;
        reg match;
        begin
            match = 1;
            for (i = 0; i < len; i = i + 1) begin
                uart_recv;
                exp_ch = expected[(len-1-i)*8 +: 8];
                $write("%c", rx_byte);
                if (rx_byte !== exp_ch) match = 0;
            end
            $write("\n");
            if (match) begin
                pass_count = pass_count + 1;
            end else begin
                $display("  *** FAIL: %0s mismatch ***", label);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // ---- Feed simulated ADC samples into the XADC sim model -------------
    integer sample_idx = 0;
    real pi = 3.14159265358979;

    task feed_samples(input integer n);
        integer k;
        real va, vb;
        begin
            for (k = 0; k < n; k = k + 1) begin
                va = 2048.0 + 500.0 * $sin(2.0 * pi * sample_idx / 32.0);
                vb = 2048.0 + 500.0 * $cos(2.0 * pi * sample_idx / 32.0);
                dut.u_xadc.sim_code_a = va;
                dut.u_xadc.sim_code_b = vb;
                sample_idx = sample_idx + 1;
                #(256 * 20);
            end
        end
    endtask

    // ---- main test sequence ---------------------------------------------
    initial begin
        $dumpfile("tb_correlator.vcd");
        $dumpvars(0, tb_correlator);

        // release testbench receiver reset
        #100; tb_rst_n = 1;

        // wait for DUT reset to release (65536 clocks = 1.31 ms)
        #(2_000_000);
        $display("=== Reset released ===");

        // -------- TEST 1: Read ID register (R00) --------
        $display("\n--- TEST 1: Read ID ---");
        uart_send("R"); uart_send("0"); uart_send("0"); uart_send(8'h0A);
        recv_and_check("434F0001\n", 9, "ID");

        #(BYTE_NS * 2);

        // -------- TEST 2: Read live ADC (R0F, R10) --------
        $display("--- TEST 2: Read live ADC A ---");
        uart_send("R"); uart_send("0"); uart_send("F"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("--- TEST 2b: Read live ADC B ---");
        uart_send("R"); uart_send("1"); uart_send("0"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        // -------- TEST 3: Set window = 100 samples, start --------
        $display("\n--- TEST 3: Set window and start ---");
        // W 02 00000064  (100 decimal = 0x64)
        uart_send("W"); uart_send("0"); uart_send("2");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("6"); uart_send("4"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "W02");

        #(BYTE_NS * 2);

        // W 01 00000001  (start)
        uart_send("W"); uart_send("0"); uart_send("1");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("1"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "Start");

        $display("  Acquisition started, feeding samples...");

        feed_samples(110);  // a few extra to be safe

        #(100_000);

        // -------- TEST 4: Read STATUS (freezes snapshot) then results --------
        $display("\n--- TEST 4: Read results ---");
        $display("  STATUS:");
        uart_send("R"); uart_send("0"); uart_send("3"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("  N:");
        uart_send("R"); uart_send("0"); uart_send("4"); uart_send(8'h0A);
        recv_and_check("00000064\n", 9, "N");

        #(BYTE_NS * 2);

        $display("  RAA_LO:");
        uart_send("R"); uart_send("0"); uart_send("5"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("  RAA_HI:");
        uart_send("R"); uart_send("0"); uart_send("6"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("  RAB_LO:");
        uart_send("R"); uart_send("0"); uart_send("9"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        // -------- TEST 5: Lag query --------
        $display("\n--- TEST 5: Lag query (lag=+8) ---");
        // W 11 00000008  (set lag = +8)
        uart_send("W"); uart_send("1"); uart_send("1");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("8"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "W11");

        #(BYTE_NS * 2);

        // W 12 00000001  (query go)
        uart_send("W"); uart_send("1"); uart_send("2");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("1"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "W12");

        #(100_000);

        $display("  XCORR_LO:");
        uart_send("R"); uart_send("1"); uart_send("3"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("  XCORR_HI:");
        uart_send("R"); uart_send("1"); uart_send("4"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        // -------- TEST 6: Error (bad write) --------
        $display("\n--- TEST 6: Bad command ---");
        uart_send("W"); uart_send("0"); uart_send("1");
        uart_send("0"); uart_send("0"); uart_send(8'h0A);
        recv_and_check("E\n", 2, "Error");

        #(BYTE_NS * 2);

        // -------- TEST 7: GPS PPS --------
        $display("\n--- TEST 7: GPS PPS ---");
        feed_samples(50);
        gps_pps = 1; #200; gps_pps = 0;
        #(10_000);
        $display("  PPS_LAST:");
        uart_send("R"); uart_send("1"); uart_send("5"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        $display("\n=== Results: %0d PASS, %0d FAIL ===", pass_count, fail_count);
        if (fail_count == 0)
            $display("=== ALL TESTS PASSED ===");
        $finish;
    end

    // safety timeout
    initial begin
        #500_000_000;
        $display("!!! TIMEOUT !!!");
        $finish;
    end

endmodule
