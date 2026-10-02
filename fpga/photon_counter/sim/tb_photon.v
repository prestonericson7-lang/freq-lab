`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// tb_photon.v -- testbench for the photon counter
//
// Uses a real uart_rx instance for receive (not manual # delays, which
// race with NBA updates in iverilog and garble data).  The loopback test
// proved both UART modules correct, so the uart_rx module is a reliable
// testbench receiver.
//
// Tests:
//   1. Reset and ID read
//   2. Set coincidence window
//   3. Start counting, inject detector pulses, stop
//   4. Read STATUS (freezes snapshot), then read counters
//   5. Verify coincidence detection
//   6. Read elapsed clock
//   7. Error handling
//   8. Fan RPM read
//
// Run with:
//   iverilog -o tb_photon \
//       sim/tb_photon.v hdl/photon_top.v hdl/coincidence.v \
//       hdl/cmd_regs_photon.v hdl/uart.v hdl/fan_pwm.v
//   vvp tb_photon
//-----------------------------------------------------------------------------
module tb_photon;

    localparam integer CLK_HZ   = 50_000_000;
    localparam integer BAUD     = 115_200;
    localparam integer CPB      = CLK_HZ / BAUD;   // 434
    localparam integer BIT_NS   = CPB * 20;         // ~8680 ns per bit
    localparam integer BYTE_NS  = BIT_NS * 10;

    reg  clk = 0;
    always #10 clk = ~clk;     // 50 MHz

    wire led_hb, led_act;
    wire fan_pwm_o;
    reg  fan_tach = 1;
    wire uart_tx;
    reg  uart_rx_pin = 1;
    reg  det_a    = 0;
    reg  det_b    = 0;
    reg  gate     = 1;         // gate always on

    photon_top #(
        .CLK_HZ          (CLK_HZ),
        .BAUD            (BAUD),
        .FAN_DUTY_DEFAULT(60)
    ) dut (
        .clk_50m  (clk),
        .led_hb   (led_hb),
        .led_act  (led_act),
        .key_n    (1'b1),
        .fan_pwm_o(fan_pwm_o),
        .fan_tach (fan_tach),
        .det_a    (det_a),
        .det_b    (det_b),
        .gate     (gate),
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
            uart_rx_pin = 0;
            #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx_pin = byte_val[i];
                #(BIT_NS);
            end
            uart_rx_pin = 1;
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

    // ---- receive and verify a hex response against expected string ------
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

    // ---- detector pulse tasks -------------------------------------------
    task pulse_a;
        begin
            det_a = 1; #60; det_a = 0;
        end
    endtask

    task pulse_b;
        begin
            det_b = 1; #60; det_b = 0;
        end
    endtask

    // ---- main test sequence ---------------------------------------------
    initial begin
        $dumpfile("tb_photon.vcd");
        $dumpvars(0, tb_photon);

        // release testbench receiver reset with DUT
        #100; tb_rst_n = 1;

        // wait for DUT reset to release (65536 clocks = 1.31 ms)
        #(2_000_000);
        $display("=== Reset released ===");

        // -------- TEST 1: Read ID (R00) --------
        $display("\n--- TEST 1: Read ID ---");
        uart_send("R"); uart_send("0"); uart_send("0"); uart_send(8'h0A);
        recv_and_check("50480001\n", 9, "ID");

        #(BYTE_NS * 2);

        // -------- TEST 2: Set coincidence window to 10 ticks (200 ns) --------
        $display("\n--- TEST 2: Set window = 10 ---");
        uart_send("W"); uart_send("0"); uart_send("2");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("A"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "W02");

        #(BYTE_NS * 2);

        // Read back window
        $display("  Read back WINDOW:");
        uart_send("R"); uart_send("0"); uart_send("2"); uart_send(8'h0A);
        recv_and_check("0000000A\n", 9, "R02");

        #(BYTE_NS * 2);

        // -------- TEST 3: Start counting --------
        $display("\n--- TEST 3: Start counting ---");
        uart_send("W"); uart_send("0"); uart_send("1");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("1"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "Start");

        // Check busy
        $display("  Busy check:");
        uart_send("R"); uart_send("0"); uart_send("3"); uart_send(8'h0A);
        uart_recv_line(9);  // status value depends on timing

        #(BYTE_NS * 2);

        // -------- TEST 4: Inject pulses --------
        $display("\n--- TEST 4: Inject pulses ---");

        // Coincident pair #1: A then B within window (100 ns gap)
        $display("  Coincident pair 1: A then B, 100ns gap");
        pulse_a;
        #100;
        pulse_b;
        #1000;

        // Coincident pair #2: B then A within window (80 ns gap)
        $display("  Coincident pair 2: B then A, 80ns gap");
        pulse_b;
        #80;
        pulse_a;
        #1000;

        // Non-coincident: A alone
        $display("  Non-coincident: A alone");
        pulse_a;
        #500;

        // Non-coincident: B alone
        $display("  Non-coincident: B alone");
        pulse_b;
        #500;

        // Coincident pair #3: simultaneous A and B
        $display("  Coincident pair 3: simultaneous");
        det_a = 1; det_b = 1; #60; det_a = 0; det_b = 0;
        #1000;

        // Non-coincident: A then B outside window (400 ns gap > 200 ns window)
        $display("  Non-coincident: A then B, 400ns gap (outside window)");
        pulse_a;
        #400;
        pulse_b;
        #1000;

        // Each pulse may be part of only ONE coincidence.
        // A at t=0, B1 at +80 ns, B2 at +180 ns (both inside A's 200 ns
        // window): one coincidence (A-B1); B2 is just a single.
        $display("  One A, two B's inside the window (must count once)");
        det_a = 1; #60; det_a = 0;
        #20; det_b = 1; #60; det_b = 0;
        #40; det_b = 1; #60; det_b = 0;
        #1000;

        // Same the other way round: B, then A1 and A2 inside B's window.
        $display("  One B, two A's inside the window (must count once)");
        det_b = 1; #60; det_b = 0;
        #20; det_a = 1; #60; det_a = 0;
        #40; det_a = 1; #60; det_a = 0;
        #1000;

        // Let elapsed clock accumulate
        #(100_000);

        // -------- TEST 5: Stop counting --------
        $display("\n--- TEST 5: Stop counting ---");
        uart_send("W"); uart_send("0"); uart_send("1");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("0"); uart_send("0");
        uart_send("0"); uart_send("2"); uart_send(8'h0A);
        recv_and_check("K\n", 2, "Stop");

        #(BYTE_NS * 2);

        // -------- TEST 6: Read results --------
        $display("\n--- TEST 6: Read results ---");

        // Read STATUS (03) -- freezes snapshot
        $display("  STATUS:");
        uart_send("R"); uart_send("0"); uart_send("3"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        // Read SINGLES_A_LO (04)
        $display("  SINGLES_A_LO:");
        uart_send("R"); uart_send("0"); uart_send("4"); uart_send(8'h0A);
        recv_and_check("00000008\n", 9, "SINGLES_A_LO");

        #(BYTE_NS * 2);

        // Read SINGLES_A_HI (05)
        $display("  SINGLES_A_HI:");
        uart_send("R"); uart_send("0"); uart_send("5"); uart_send(8'h0A);
        recv_and_check("00000000\n", 9, "SINGLES_A_HI");

        #(BYTE_NS * 2);

        // Read SINGLES_B_LO (06)
        $display("  SINGLES_B_LO:");
        uart_send("R"); uart_send("0"); uart_send("6"); uart_send(8'h0A);
        recv_and_check("00000008\n", 9, "SINGLES_B_LO");

        #(BYTE_NS * 2);

        // Read SINGLES_B_HI (07)
        $display("  SINGLES_B_HI:");
        uart_send("R"); uart_send("0"); uart_send("7"); uart_send(8'h0A);
        recv_and_check("00000000\n", 9, "SINGLES_B_HI");

        #(BYTE_NS * 2);

        // Read COINC_LO (08) -- expecting 5 coincidences: pairs 1, 2 and 3
        // (3 = simultaneous) plus one from each A,B,B / B,A,A group
        $display("  COINC_LO (expect 5):");
        uart_send("R"); uart_send("0"); uart_send("8"); uart_send(8'h0A);
        recv_and_check("00000005\n", 9, "COINC_LO");

        #(BYTE_NS * 2);

        // Read COINC_HI (09)
        $display("  COINC_HI:");
        uart_send("R"); uart_send("0"); uart_send("9"); uart_send(8'h0A);
        recv_and_check("00000000\n", 9, "COINC_HI");

        #(BYTE_NS * 2);

        // Read ELAPSED (0A)
        $display("  ELAPSED:");
        uart_send("R"); uart_send("0"); uart_send("A"); uart_send(8'h0A);
        uart_recv_line(9);

        #(BYTE_NS * 2);

        // -------- TEST 7: Error handling --------
        $display("\n--- TEST 7: Bad command ---");
        uart_send("X"); uart_send("1"); uart_send("2"); uart_send(8'h0A);
        recv_and_check("E\n", 2, "Error");

        #(BYTE_NS * 2);

        // -------- TEST 8: Read fan RPM (0B) --------
        $display("\n--- TEST 8: Fan RPM ---");
        uart_send("R"); uart_send("0"); uart_send("B"); uart_send(8'h0A);
        uart_recv_line(9);

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
