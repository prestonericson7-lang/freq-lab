`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// tb_trng.v -- testbench for the TRNG pipeline (entropy_src sim backdoor +
// trng_core + cmd_regs_trng). Feeds a raw-bit stream from +STIM, logs every
// output word and the final register reads to +LOG, and reads words/counters
// back over the UART. check_trng.py compares to trng_model.py.
//
//   iverilog -DSIMULATION -o tb_trng tb_trng.v ../hdl/entropy_src.v ../hdl/trng_core.v \
//            ../hdl/cmd_regs_trng.v ../hdl/uart.v
//   vvp tb_trng +STIM=stim_balanced.txt +LOG=log_balanced.txt
//-----------------------------------------------------------------------------
module tb_trng;
    localparam integer CLK_HZ = 50_000_000;
    localparam integer BAUD   = 1_000_000;
    localparam integer CPB    = CLK_HZ / BAUD;   // 50
    localparam integer BIT_NS = CPB * 20;

    reg clk = 1'b0; always #10 clk = ~clk;
    reg rst_n = 1'b0;
    initial begin #200 rst_n = 1'b1; end

    // config read from the stim file
    integer rct_cut, apt_win, apt_cut;

    // entropy source (sim mode) driven by the testbench
    reg       sim_bit = 1'b0, sim_stb = 1'b0;
    wire      raw_bit, raw_stb;
    entropy_src #(.NOSC(16), .RLEN(3), .DECIM(8)) u_src (
        .clk(clk), .rst_n(rst_n), .en(1'b1),
        .sim_bit(sim_bit), .sim_stb(sim_stb), .raw_bit(raw_bit), .raw_stb(raw_stb));

    // register-file outputs that drive the core
    wire        run, clear_alarms;
    wire [5:0]  rct_cutoff; wire [15:0] apt_window, apt_cutoff; wire [7:0] fan_duty;

    // core
    wire [31:0] word; wire word_stb;
    wire rct_fail, apt_fail;
    wire [31:0] raw_count, vn_count, rct_fail_count, apt_fail_count;
    trng_core u_core (
        .clk(clk), .rst_n(rst_n), .raw_bit(raw_bit), .raw_stb(raw_stb),
        .clear_alarms(clear_alarms), .rct_cutoff(rct_cutoff), .apt_window(apt_window), .apt_cutoff(apt_cutoff),
        .word(word), .word_stb(word_stb),
        .rct_fail(rct_fail), .apt_fail(apt_fail),
        .raw_count(raw_count), .vn_count(vn_count),
        .rct_fail_count(rct_fail_count), .apt_fail_count(apt_fail_count));

    reg [31:0] word_count;
    always @(posedge clk) if (!rst_n) word_count<=0; else if (word_stb) word_count<=word_count+1;

    // register file + UART
    wire uart_tx; reg uart_rx_pin = 1'b1;
    cmd_regs_trng #(.CLKS_PER_BIT(CPB), .FAN_DUTY_DEFAULT(60)) u_regs (
        .clk(clk), .rst_n(rst_n), .rx(uart_rx_pin), .tx(uart_tx),
        .run(run), .clear_alarms(clear_alarms),
        .rct_cutoff(rct_cutoff), .apt_window(apt_window), .apt_cutoff(apt_cutoff), .fan_duty(fan_duty),
        .word_in(word), .word_stb(word_stb),
        .rct_fail(rct_fail), .apt_fail(apt_fail),
        .raw_count(raw_count), .vn_count(vn_count),
        .rct_fail_count(rct_fail_count), .apt_fail_count(apt_fail_count), .word_count(word_count),
        .temp_code(12'h97C), .vccint_code(12'h555), .fan_rpm(16'd1234));

    // log file
    integer flog;
    always @(posedge clk) if (word_stb) $fdisplay(flog, "W %08h", word);

    // UART helpers (TB-side)
    reg tbrx_rst = 1'b0; wire [7:0] tbrx_data; wire tbrx_valid;
    uart_rx #(.CLKS_PER_BIT(CPB)) u_tbrx (.clk(clk), .rst_n(tbrx_rst), .rx(uart_tx), .data(tbrx_data), .valid(tbrx_valid));
    initial #250 tbrx_rst = 1'b1;
    reg [7:0] rxq [0:1023]; integer rx_wp=0, rx_rp=0;
    always @(posedge clk) if (tbrx_valid) begin rxq[rx_wp%1024]=tbrx_data; rx_wp=rx_wp+1; end

    task uart_send(input [7:0] b); integer i; begin
        uart_rx_pin=1'b0; #(BIT_NS);
        for (i=0;i<8;i=i+1) begin uart_rx_pin=b[i]; #(BIT_NS); end
        uart_rx_pin=1'b1; #(BIT_NS);
    end endtask
    task uart_get(output [7:0] b); integer g; begin
        g=0; while (rx_rp==rx_wp && g<400000) begin @(posedge clk); g=g+1; end
        if (rx_rp==rx_wp) b=8'h00; else begin b=rxq[rx_rp%1024]; rx_rp=rx_rp+1; end
    end endtask
    function [7:0] hexc(input [3:0] n); hexc=(n<10)?(8'h30+n):(8'h37+n); endfunction
    function [3:0] hexv(input [7:0] c); hexv=(c<="9")?c[3:0]:(c[3:0]+4'd9); endfunction

    reg [31:0] rv; reg rerr;
    task read_reg(input [7:0] a, output [31:0] v, output err); integer i; reg [7:0] c; begin
        uart_send("R"); uart_send(hexc(a[7:4])); uart_send(hexc(a[3:0])); uart_send(8'h0A);
        v=32'd0; err=1'b0; uart_get(c); v={28'd0,hexv(c)}; uart_get(c);
        if (v==32'hE && c==8'h0A) err=1'b1;
        else begin v={v[27:0],hexv(c)}; for(i=2;i<8;i=i+1) begin uart_get(c); v={v[27:0],hexv(c)}; end uart_get(c); if (c!=8'h0A) err=1'b1; end
    end endtask
    task write_reg(input [7:0] a, input [31:0] d, output err); integer i; reg [7:0] c; begin
        uart_send("W"); uart_send(" "); uart_send(hexc(a[7:4])); uart_send(hexc(a[3:0])); uart_send(" ");
        for (i=7;i>=0;i=i-1) uart_send(hexc(d[i*4 +: 4])); uart_send(8'h0A);
        uart_get(c); err=(c!="K"); uart_get(c); if (c!=8'h0A) err=1'b1;
    end endtask

    integer pass=0, fail=0;
    task check(input cond, input [8*64-1:0] what); begin
        if (cond) begin pass=pass+1; $display("  PASS  %0s", what); end
        else begin fail=fail+1; $display("  FAIL  %0s", what); end
    end endtask

    // config comes from the register file; but the core reads apt_window etc. from
    // the register file, so set them over the UART before feeding bits.
    integer fstim, ch, nbits;
    reg [8*64-1:0] stim_name, log_name;
    reg [31:0] v1, v2, v3, v4, v5, v6, v7, v8;
    reg e;
    integer k;
    initial begin
        if (!$value$plusargs("STIM=%s", stim_name)) stim_name = "stim_balanced.txt";
        if (!$value$plusargs("LOG=%s", log_name))   log_name  = "log_balanced.txt";
        flog = $fopen(log_name, "w");
        fstim = $fopen(stim_name, "r");
        if (fstim == 0) begin $display("cannot open stim"); $finish; end
        k = $fscanf(fstim, "%d %d %d\n", rct_cut, apt_win, apt_cut);

        @(posedge rst_n); #1000;
        // program the config
        write_reg(8'h09, rct_cut, e);
        write_reg(8'h0A, apt_win, e);
        write_reg(8'h0B, apt_cut, e);

        // feed the raw bits, one every 16 clocks
        nbits = 0;
        ch = $fgetc(fstim);
        while (ch == "0" || ch == "1") begin
            @(posedge clk); sim_bit <= (ch == "1"); sim_stb <= 1'b1;
            @(posedge clk); sim_stb <= 1'b0;
            repeat (14) @(posedge clk);
            nbits = nbits + 1;
            ch = $fgetc(fstim);
        end
        $fclose(fstim);
        // drain the pipeline
        repeat (200) @(posedge clk);

        // read the counters and health
        read_reg(8'h00, v1, e); check(!e && v1==32'h54524E31, "ID = 54524E31");
        read_reg(8'h04, v2, e);           // raw_count
        read_reg(8'h05, v3, e);           // vn_count
        read_reg(8'h06, v4, e);           // rct_fails
        read_reg(8'h07, v5, e);           // apt_fails
        read_reg(8'h08, v6, e);           // word_count
        read_reg(8'h02, v7, e);           // status
        $fdisplay(flog, "R %0d %08h %08h %08h %08h %08h %08h %0d %0d %0d",
                  nbits, v2, v3, v4, v5, v6, v7, rct_cut, apt_win, apt_cut);
        check(v2 == raw_count, "raw_count register matches the core");
        check(v6 == word_count, "word_count register matches the core");

        // pop up to 64 words from the FIFO and log them (DATA reads)
        for (k = 0; k < 64; k = k + 1) begin
            read_reg(8'h02, v7, e);
            if ((v7 & 32'h8) != 0) k = 64;      // FIFO empty
            else begin read_reg(8'h03, v8, e); $fdisplay(flog, "D %08h", v8); end
        end

        $display("=== %0d PASS, %0d FAIL ===", pass, fail);
        $fdisplay(flog, "END");
        $fclose(flog);
        $finish;
    end

    initial begin #2_000_000_000; $display("*** TIMEOUT ***"); $finish; end
endmodule
