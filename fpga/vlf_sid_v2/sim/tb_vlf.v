`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// tb_vlf.v -- testbench for the VLF SID receiver (vlf_top)
//
// Feeds stimulus.hex (made by gen_files.py) into the XADC simulation model,
// drives a fake GPS PPS (edges at 5, 55, 105, 155, 205 ms) and talks to the
// design over its UART at 1 Mbaud (115200 in hardware; the UART itself is the
// same module the other designs already use).
//
// Self-checks here: register defaults, writes, error replies, snapshot
// freezing, PPS-aligned and free-running windows, clip counting, PPS count.
// Everything numeric (every decimated I/Q value of every channel and every
// window sum) is written to sim_log.txt and checked bit-for-bit, and against
// an ideal floating-point receiver, by check_vlf.py.
//
//   iverilog -DSIMULATION -o tb_vlf sim/tb_vlf.v hdl/*.v
//   cd sim && vvp ../tb_vlf && python3 check_vlf.py
//-----------------------------------------------------------------------------
module tb_vlf;

    localparam integer CLK_HZ = 50_000_000;
    localparam integer BAUD   = 1_000_000;
    localparam integer CPB    = CLK_HZ / BAUD;      // 50 clocks per bit
    localparam integer BIT_NS = CPB * 20;
    localparam integer NSTIM  = 90_000;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg  gps_pps = 1'b0;
    reg  uart_rx_pin = 1'b1;
    wire uart_tx;
    wire led_hb, led_act, fan_pwm_o;

    vlf_top #(.CLK_HZ(CLK_HZ), .BAUD(BAUD), .FAN_DUTY_DEFAULT(60)) dut (
        .clk_50m(clk), .led_hb(led_hb), .led_act(led_act), .key_n(1'b1),
        .fan_pwm_o(fan_pwm_o), .fan_tach(1'b1),
        .vauxp1(1'b0), .vauxn1(1'b0),
        .gps_pps(gps_pps), .uart_tx(uart_tx), .uart_rx(uart_rx_pin));

    // ------------------------------------------------ stimulus into the XADC model
    reg [11:0] stim [0:NSTIM-1];
    integer    sidx = 0;
    initial begin
        $readmemh("stimulus.hex", stim);
        #1 dut.u_xadc.sim_code = stim[0];
    end
    always @(posedge clk) begin
        if (dut.u_xadc.stb) begin
            sidx = sidx + 1;
            if (sidx < NSTIM) dut.u_xadc.sim_code <= stim[sidx];
        end
    end

    // ------------------------------------------------ fake GPS PPS
    initial begin
        #(5_000_000);
        repeat (5) begin
            gps_pps = 1'b1; #(10_000_000);
            gps_pps = 1'b0; #(40_000_000);
        end
    end

    // ------------------------------------------------ logs for check_vlf.py
    integer flog;
    reg     close_d = 1'b0;
    integer kk;
    initial flog = $fopen("sim_log.txt", "w");

    // sferic capture: the PPS edge as the design sees it, and every trigger
    // (the strobe comes one clock after the trigger sample's a_valid)
    always @(posedge clk) begin
        if (dut.pps_rise)  $fdisplay(flog, "G %0d", $time);
        if (dut.u_sf.trig) $fdisplay(flog, "T %0d %0d", sidx - 1, $time - 20);
    end

    always @(posedge clk) begin
        if (dut.a_valid)
            $fdisplay(flog, "S %0d %0d %0d %0d %0d %0d", dut.a_code, dut.a_dec,
                      dut.a_close, dut.a_run, dut.a_close_pps, dut.a_close_to);
        if (dut.iq_valid_bus[0])
            $fdisplay(flog, "D %h %h", dut.dbg_i_bus, dut.dbg_q_bus);
        close_d <= dut.a_valid && dut.a_close;
        if (close_d) begin
            $fdisplay(flog, "W %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d %0d",
                      dut.seq, dut.w_n, dut.w_ndec, dut.w_clip, dut.w_min, dut.w_max,
                      dut.w_sq, $signed(dut.w_sum), dut.w_start_pps, dut.w_end_pps, dut.w_end_to);
            for (kk = 0; kk < 8; kk = kk + 1)
                $fdisplay(flog, "C %0d %0d %0d", kk,
                          dut.w_pow_bus[kk*64 +: 64], dut.w_pmax_bus[kk*48 +: 48]);
        end
    end

    // ------------------------------------------------ UART: send bytes, receive into a queue
    task uart_send(input [7:0] b);
        integer i;
        begin
            uart_rx_pin = 1'b0; #(BIT_NS);
            for (i = 0; i < 8; i = i + 1) begin uart_rx_pin = b[i]; #(BIT_NS); end
            uart_rx_pin = 1'b1; #(BIT_NS);
        end
    endtask

    reg        tbrx_rst_n = 1'b0;
    wire [7:0] tbrx_data;
    wire       tbrx_valid;
    uart_rx #(.CLKS_PER_BIT(CPB)) u_tbrx (
        .clk(clk), .rst_n(tbrx_rst_n), .rx(uart_tx), .data(tbrx_data), .valid(tbrx_valid));
    initial #200 tbrx_rst_n = 1'b1;

    reg [7:0] rxq [0:1023];
    integer   rx_wp = 0, rx_rp = 0;
    always @(posedge clk) if (tbrx_valid) begin rxq[rx_wp % 1024] = tbrx_data; rx_wp = rx_wp + 1; end

    task uart_get(output [7:0] b);
        integer guard;
        begin
            guard = 0;
            while (rx_rp == rx_wp && guard < 200_000) begin @(posedge clk); guard = guard + 1; end
            if (rx_rp == rx_wp) begin b = 8'h00; $display("  *** UART receive timeout"); end
            else begin b = rxq[rx_rp % 1024]; rx_rp = rx_rp + 1; end
        end
    endtask

    function [7:0] hexc(input [3:0] n);
        hexc = (n < 10) ? (8'h30 + n) : (8'h37 + n);
    endfunction
    function [3:0] hexv(input [7:0] c);
        hexv = (c <= "9") ? c[3:0] : (c[3:0] + 4'd9);
    endfunction

    integer pass_count = 0, fail_count = 0;
    task check(input cond, input [8*72-1:0] what);
        begin
            if (cond) begin pass_count = pass_count + 1; $display("  PASS  %0s", what); end
            else      begin fail_count = fail_count + 1; $display("  FAIL  %0s", what); end
        end
    endtask

    reg  [31:0] rv;
    reg         rerr;
    task read_reg(input [7:0] a, output [31:0] v, output err);
        integer i;
        reg [7:0] c;
        begin
            uart_send("R"); uart_send(hexc(a[7:4])); uart_send(hexc(a[3:0])); uart_send(8'h0A);
            v = 32'd0; err = 1'b0;
            uart_get(c);
            v = {28'd0, hexv(c)};
            uart_get(c);
            // "E" followed by a newline is the error reply; "E" followed by a hex
            // digit is a value that happens to start with the digit E (0xE...)
            if (v == 32'hE && c == 8'h0A) begin
                err = 1'b1;
            end else begin
                v = {v[27:0], hexv(c)};
                for (i = 2; i < 8; i = i + 1) begin uart_get(c); v = {v[27:0], hexv(c)}; end
                uart_get(c);
                if (c != 8'h0A) err = 1'b1;
            end
        end
    endtask

    task write_reg(input [7:0] a, input [31:0] d, output err);
        integer i;
        reg [7:0] c;
        begin
            uart_send("W"); uart_send(" ");
            uart_send(hexc(a[7:4])); uart_send(hexc(a[3:0])); uart_send(" ");
            for (i = 7; i >= 0; i = i - 1) uart_send(hexc(d[i*4 +: 4]));
            uart_send(8'h0A);
            uart_get(c);
            err = (c != "K");
            uart_get(c);
            if (c != 8'h0A) err = 1'b1;
        end
    endtask

    // read STATUS (takes the snapshot) and every snapshot register; log them
    reg [31:0] snap [0:63];
    task read_snapshot(input integer tag);
        integer r;
        reg e;
        begin
            read_reg(8'h03, snap[3], e);
            for (r = 4; r <= 8'h0D; r = r + 1) read_reg(r[7:0], snap[r], e);
            for (r = 0; r < 8; r = r + 1) begin
                read_reg(8'h20 + 4*r, snap[32 + 4*r], e);
                read_reg(8'h21 + 4*r, snap[33 + 4*r], e);
                read_reg(8'h22 + 4*r, snap[34 + 4*r], e);
            end
            $fdisplay(flog, "R %0d %h %h %h %h %h %h %h %h %h %h %h", tag,
                      snap[3], snap[4], snap[5], snap[6], snap[7], snap[8],
                      snap[9], snap[10], snap[11], snap[12], snap[13]);
            for (r = 0; r < 8; r = r + 1)
                $fdisplay(flog, "P %0d %0d %h %h %h", tag, r,
                          snap[33 + 4*r], snap[32 + 4*r], snap[34 + 4*r]);
            $display("  snapshot %0d: SEQ=%0d NSAMP=%0d NDEC=%0d FLAGS=%0d CLIP=%0d XMINMAX=%h",
                     tag, snap[4], snap[5], snap[6], snap[7], snap[8], snap[9]);
        end
    endtask

    // the same 35 reads sent back-to-back without waiting (how vlf_host.py reads):
    // the design queues the replies; they must equal the one-at-a-time reads
    reg [7:0] baddr [0:34];
    task burst_snapshot(output ok);
        integer i, j;
        reg [7:0]  c;
        reg [31:0] v;
        begin
            baddr[0] = 8'h03;
            for (i = 0; i < 10; i = i + 1) baddr[1 + i] = 8'h04 + i;
            for (i = 0; i < 8; i = i + 1) begin
                baddr[11 + 3*i] = 8'h20 + 4*i;
                baddr[12 + 3*i] = 8'h21 + 4*i;
                baddr[13 + 3*i] = 8'h22 + 4*i;
            end
            for (i = 0; i < 35; i = i + 1) begin
                uart_send("R"); uart_send(hexc(baddr[i][7:4])); uart_send(hexc(baddr[i][3:0]));
                uart_send(8'h0A);
            end
            ok = 1'b1;
            for (i = 0; i < 35; i = i + 1) begin
                v = 32'd0;
                for (j = 0; j < 8; j = j + 1) begin uart_get(c); v = {v[27:0], hexv(c)}; end
                uart_get(c);
                if (c != 8'h0A) ok = 1'b0;
                if (i == 0) begin if (v != snap[3]) ok = 1'b0; end
                else if (i <= 10) begin if (v != snap[3 + i]) ok = 1'b0; end
                else if (v != snap[32 + 4*((i - 11) / 3) + ((i - 11) % 3)]) ok = 1'b0;
            end
        end
    endtask

    task wait_ms(input real t_ms);
        begin
            if ($realtime < t_ms * 1.0e6) #(t_ms * 1.0e6 - $realtime);
        end
    endtask

    // read the oldest sferic event (header + 32 snapshot words), log it, pop it
    reg [31:0] evh [0:3];
    reg [31:0] evs [0:31];
    task read_event(input integer tag, output [31:0] nev_before, output [31:0] popped);
        integer r;
        reg e;
        begin
            read_reg(8'h43, nev_before, e);
            for (r = 0; r < 4; r = r + 1) read_reg(8'h45 + r, evh[r], e);
            for (r = 0; r < 32; r = r + 1) read_reg(8'h60 + r, evs[r], e);
            $fwrite(flog, "E %0d %0d %h %h %h %h", tag, nev_before, evh[0], evh[1], evh[2], evh[3]);
            for (r = 0; r < 32; r = r + 1) $fwrite(flog, " %h", evs[r]);
            $fdisplay(flog, "");
            read_reg(8'h44, popped, e);
            $display("  event %0d: PPS=%0d CLK=%0d (pps seen %0d) peak=%0d pol=%0d seq=%0d",
                     tag, evh[0], evh[1][25:0], evh[1][31], evh[2][11:0], evh[2][12], evh[3]);
        end
    endtask

    // ------------------------------------------------ test sequence
    reg [31:0] pow0_a, v1, v2;
    reg        e;
    integer    r;
    reg [31:0] ftw_def [0:7];
    initial begin
        ftw_def[0] = 32'h1040BFE4; ftw_def[1] = 32'h0E065300;
        ftw_def[2] = 32'h0FBA8827; ftw_def[3] = 32'h1083DBC2;
        ftw_def[4] = 32'h1AB4B72C; ftw_def[5] = 32'h0CF9E386;
        ftw_def[6] = 32'h0E8C8ABD; ftw_def[7] = 32'h13A92A30;

        wait_ms(1.5);                 // design reset releases after 65536 clocks = 1.31 ms
        $display("=== VLF SID receiver testbench ===");

        read_reg(8'h00, rv, rerr); check(!rerr && rv == 32'h564C0002, "ID = 564C0002 (v2, sferic capture)");
        read_reg(8'h01, rv, rerr); check(!rerr && rv == 32'd3,        "CTRL default = RUN|PPS_MODE");
        read_reg(8'h02, rv, rerr); check(!rerr && rv == 32'd390625,   "WIN default = 390625");
        read_reg(8'h18, rv, rerr); check(!rerr && rv == 32'd60,       "FAN_DUTY default = 60");
        e = 1'b0;
        for (r = 0; r < 8; r = r + 1) begin
            read_reg(8'h10 + r, rv, rerr);
            if (rerr || rv !== ftw_def[r]) e = 1'b1;
        end
        check(!e, "FTW0..7 defaults (8 stations)");

        write_reg(8'h02, 32'd19531, rerr); check(!rerr, "write WIN = 19531 -> K");
        read_reg(8'h02, rv, rerr);         check(!rerr && rv == 32'd19531, "WIN reads back 19531");

        // sferic capture defaults, then arm it at 600 counts (signals peak near 423)
        read_reg(8'h40, rv, rerr); check(!rerr && rv == 32'd0,    "SF_THRESH default = 0 (off)");
        read_reg(8'h41, rv, rerr); check(!rerr && rv == 32'd1953, "SF_HOLDOFF default = 1953 (5 ms)");
        read_reg(8'h42, rv, rerr); check(!rerr && rv == 32'd0,    "SF_COUNT = 0 at start");
        read_reg(8'h43, rv, rerr); check(!rerr && rv == 32'd0,    "SF_NEV = 0 at start");
        write_reg(8'h41, 32'd1000, rerr);  check(!rerr, "write SF_HOLDOFF = 1000 -> K");
        read_reg(8'h41, rv, rerr);         check(!rerr && rv == 32'd1000, "SF_HOLDOFF reads back 1000");
        write_reg(8'h40, 32'd600, rerr);   check(!rerr, "write SF_THRESH = 600 -> K");
        read_reg(8'h40, rv, rerr);         check(!rerr && rv == 32'd600, "SF_THRESH reads back 600");
        write_reg(8'h45, 32'd1, rerr);     check(rerr, "write to read-only EV_PPS -> E");

        // error handling
        write_reg(8'h05, 32'd1, rerr);     check(rerr, "write to read-only NSAMP -> E");
        uart_send("X"); uart_send(8'h0A);
        uart_get(v1[7:0]); uart_get(v2[7:0]);
        check(v1[7:0] == "E" && v2[7:0] == 8'h0A, "garbage command -> E");
        uart_send("R"); uart_send("3"); uart_send(8'h0A);
        uart_get(v1[7:0]); uart_get(v2[7:0]);
        check(v1[7:0] == "E" && v2[7:0] == 8'h0A, "read with one address digit -> E");

        // ---- window 3 (55..105 ms): clean signal, PPS at both ends
        wait_ms(107.0);
        read_snapshot(3);
        check(snap[4] == 32'd3,                        "snapshot SEQ = 3");
        check(snap[5] == 32'd19531 || snap[5] == 32'd19532, "NSAMP = 19531/19532 (50 ms of PPS)");
        check(snap[6] == 32'd19 || snap[6] == 32'd20,  "NDEC = 19/20");
        check(snap[7] == 32'd3,                        "FLAGS: started and ended on PPS");
        check(snap[8] == 32'd0,                        "CLIP = 0 in a clean window");
        check(snap[3][3] == 1'b1,                      "STATUS: PPS alive");
        pow0_a = snap[32];
        burst_snapshot(e);
        check(e, "35 back-to-back reads (queued replies) == one-at-a-time reads");

        // snapshot must stay frozen until STATUS is read again
        wait_ms(150.0);
        read_reg(8'h20, rv, rerr); check(!rerr && rv == pow0_a, "POW0 frozen mid-window");
        wait_ms(157.0);
        read_reg(8'h20, rv, rerr); check(!rerr && rv == pow0_a, "POW0 still frozen after a window closed");

        // ---- window 4 (105..155 ms): has the clipping burst
        read_snapshot(4);
        check(snap[4] == 32'd4,                        "snapshot SEQ = 4");
        check(snap[8] == 32'd8,                        "CLIP = 8 (five 4095s + three 0s)");
        check(snap[9] == 32'h0FFF0000,                 "XMINMAX = max 4095, min 0");
        check(snap[32] != pow0_a,                      "new snapshot replaced POW0");

        // ---- free-running windows of 20000 samples
        write_reg(8'h02, 32'd20000, rerr); check(!rerr, "write WIN = 20000");
        write_reg(8'h01, 32'd1, rerr);     check(!rerr, "write CTRL = RUN only (PPS ignored)");
        wait_ms(208.0);
        read_snapshot(5);
        check(snap[4] == 32'd5,                        "snapshot SEQ = 5");
        check(snap[5] == 32'd20000,                    "free-run NSAMP = 20000 exactly");
        check(snap[7] == 32'd2,                        "FLAGS: started on PPS, ended on count");

        read_reg(8'h0F, rv, rerr); check(!rerr && rv == 32'd5, "PPS_CNT = 5");
        read_reg(8'h0E, rv, rerr); check(!rerr && rv < 32'd4096, "live ADC readable");
        read_reg(8'h1A, rv, rerr); check(!rerr && rv == 32'd8388608, "FS_CLKS = 8388608 (128 clocks/sample)");

        // ---- sferic events: the two strokes and the clipping burst = 3 triggers.
        // Exact timestamps and snapshots are checked against the stimulus by check_vlf.py.
        read_reg(8'h42, rv, rerr); check(!rerr && rv == 32'd3, "SF_COUNT = 3 (two strokes + the clip burst)");
        read_reg(8'h43, rv, rerr); check(!rerr && rv == 32'd3, "SF_NEV = 3 waiting");
        read_reg(8'h49, rv, rerr); check(!rerr && rv == 32'd0, "SF_LOST = 0");
        read_event(0, v1, v2);
        check(v1 == 32'd3 && v2 == 32'd3,                     "event 0: 3 waiting, pop returned 3");
        check(evh[0] == 32'd3 && evh[1][31] == 1'b1,          "event 0: GPS second 3, PPS seen");
        check(evh[2][31:16] == 16'h1040 && evh[2][12] == 1'b0, "event 0: PRE 16 / LEN 64, positive stroke");
        check(evh[3] == 32'd0,                                "event 0: SEQ 0");
        read_event(1, v1, v2);
        check(v1 == 32'd2 && v2 == 32'd2,                     "event 1: 2 waiting, pop returned 2");
        check(evh[0] == 32'd3 && evh[2][12:0] == 13'h1800,    "event 1: clip burst, |peak| = 2048 negative, GPS second 3");
        check(evh[3] == 32'd1,                                "event 1: SEQ 1");
        read_event(2, v1, v2);
        check(v1 == 32'd1 && v2 == 32'd1,                     "event 2: 1 waiting, pop returned 1");
        check(evh[0] == 32'd4 && evh[2][12] == 1'b1,          "event 2: GPS second 4, negative stroke");
        check(evh[3] == 32'd2,                                "event 2: SEQ 2");
        read_reg(8'h43, rv, rerr); check(!rerr && rv == 32'd0, "SF_NEV = 0 after three pops");
        read_reg(8'h44, rv, rerr); check(!rerr && rv == 32'd0, "pop on an empty FIFO returns 0");
        read_reg(8'h43, rv, rerr); check(!rerr && rv == 32'd0, "... and leaves SF_NEV = 0");
        read_reg(8'h42, rv, rerr); check(!rerr && rv == 32'd3, "SF_COUNT still 3");

        $fdisplay(flog, "END");
        $fflush(flog);

        // FTW write / read back (after the log ends: it changes the NCO)
        write_reg(8'h17, 32'h12345678, rerr); check(!rerr, "write FTW7");
        read_reg(8'h17, rv, rerr);            check(!rerr && rv == 32'h12345678, "FTW7 reads back");

        $display("=== %0d PASS, %0d FAIL ===", pass_count, fail_count);
        $fclose(flog);
        $finish;
    end

    initial begin
        #(260_000_000);
        $display("*** TIMEOUT ***");
        $finish;
    end

endmodule
