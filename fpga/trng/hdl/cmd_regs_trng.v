`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cmd_regs_trng.v -- UART register file for the ring-oscillator TRNG (#73).
// Same text protocol as the other freq-lab designs (115200 8N1):
//   W aa dddddddd\n  write -> "K\n"   R aa\n  read -> "dddddddd\n"   else "E\n"
// Replies go through a 512-byte FIFO.
//
// Register map:
//   00 ID        RO  0x54524E31 ("TRN1")
//   01 CTRL      RW  bit0 RUN (enable the noise source). Writing with bit8=1
//                    also clears the sticky health alarms.                 default 0x1
//   02 STATUS    RO  bit0 RUN, bit1 RCT alarm, bit2 APT alarm, bit3 FIFO empty,
//                    bit4 FIFO full, [15:8] words available in the FIFO
//   03 DATA      RO  a 32-bit random word; READING IT POPS THE FIFO. 0 if empty
//                    (bit5 of STATUS, underflow sticky, set then; cleared by CTRL clear)
//   04 RAW_CNT   RO  raw noise-source bits seen
//   05 VN_CNT    RO  von Neumann output bits
//   06 RCT_FAILS RO  Repetition Count Test alarms raised
//   07 APT_FAILS RO  Adaptive Proportion Test alarms raised
//   08 WORD_CNT  RO  32-bit words produced
//   09 RCT_CUT   RW  RCT cutoff (repeats in a row -> alarm)              default 20
//   0A APT_WIN   RW  APT window length (raw bits)                         default 1024
//   0B APT_CUT   RW  APT cutoff (count equal to the window's first bit)   default 660
//   0C DIE_TEMP  RO  XADC on-chip temp code (T_C = code*503.975/4096 - 273.15)
//   0D VCCINT    RO  XADC VCCINT code (V = code*3.0/4096)
//   18 FAN_DUTY  RW  0..100 %                                             default 60
//   19 FAN_RPM   RO
//-----------------------------------------------------------------------------
module cmd_regs_trng #(
    parameter integer CLKS_PER_BIT     = 434,
    parameter integer FAN_DUTY_DEFAULT = 60
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output wire        tx,

    output reg         run,
    output reg         clear_alarms,       // one-clock pulse
    output reg  [5:0]  rct_cutoff,
    output reg  [15:0] apt_window,
    output reg  [15:0] apt_cutoff,
    output reg  [7:0]  fan_duty,

    input  wire [31:0] word_in,
    input  wire        word_stb,
    input  wire        rct_fail,
    input  wire        apt_fail,
    input  wire [31:0] raw_count,
    input  wire [31:0] vn_count,
    input  wire [31:0] rct_fail_count,
    input  wire [31:0] apt_fail_count,
    input  wire [31:0] word_count,
    input  wire [11:0] temp_code,
    input  wire [11:0] vccint_code,
    input  wire [15:0] fan_rpm
);
    localparam [31:0] ID_WORD = 32'h54524E31;

    wire [7:0] rx_data; wire rx_valid;
    reg  [7:0] tx_data; reg tx_send; wire tx_busy;
    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (.clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_data), .valid(rx_valid));
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (.clk(clk), .rst_n(rst_n), .data(tx_data), .send(tx_send), .tx(tx), .busy(tx_busy));

    function is_hex(input [7:0] c); is_hex = ((c>="0")&&(c<="9"))||((c>="A")&&(c<="F"))||((c>="a")&&(c<="f")); endfunction
    function [3:0] hex_val(input [7:0] c); if (c<="9") hex_val=c[3:0]; else hex_val=c[3:0]+4'd9; endfunction
    function [7:0] hex_chr(input [3:0] n); hex_chr=(n<4'd10)?(8'h30+{4'd0,n}):(8'h37+{4'd0,n}); endfunction

    // ------------------------------------------------ word FIFO (64 x 32)
    reg  [31:0] fifo [0:63];
    reg  [6:0]  wp, rp;
    wire        f_empty = (wp == rp);
    wire        f_full  = (wp[5:0] == rp[5:0]) && (wp[6] != rp[6]);
    wire [6:0]  f_count = wp - rp;
    reg         pop;
    reg         underflow;

    always @(posedge clk) begin
        if (!rst_n) begin wp <= 7'd0; underflow <= 1'b0; end
        else begin
            if (word_stb && !f_full) begin fifo[wp[5:0]] <= word_in; wp <= wp + 7'd1; end
            if (clear_alarms) underflow <= 1'b0;
            if (pop && f_empty) underflow <= 1'b1;
        end
    end
    reg [31:0] pop_data;
    always @(posedge clk) begin
        if (!rst_n) rp <= 7'd0;
        else if (pop && !f_empty) rp <= rp + 7'd1;
    end
    always @(*) pop_data = f_empty ? 32'd0 : fifo[rp[5:0]];

    // ------------------------------------------------ parser / transmitter state
    reg [1:0] pstate; localparam P_IDLE=2'd0, P_HEX=2'd1, P_ERR=2'd2;
    reg cmd_w; reg [3:0] nib; reg [39:0] sr;
    reg [71:0] txbuf; reg [3:0] txcnt;
    reg [7:0] fifo_tx [0:511];
    reg [9:0] t_wp, t_rp;
    wire tf_empty = (t_wp == t_rp);
    wire tf_full  = (t_wp[8:0] == t_rp[8:0]) && (t_wp[9] != t_rp[9]);
    reg [1:0] tstate; localparam T_IDLE=2'd0, T_WAIT1=2'd1, T_WAIT2=2'd2;

    wire [7:0]  w_addr = sr[39:32];
    wire [31:0] w_data = sr[31:0];
    wire [7:0]  r_addr = sr[7:0];
    wire        w_ok = (w_addr==8'h01)||(w_addr==8'h09)||(w_addr==8'h0A)||(w_addr==8'h0B)||(w_addr==8'h18);
    wire is_eol = (rx_data==8'h0A)||(rx_data==8'h0D);

    reg [31:0] rdata;
    always @(*) begin
        case (r_addr)
            8'h00: rdata = ID_WORD;
            8'h01: rdata = {31'd0, run};
            8'h02: rdata = {16'd0, 1'b0, f_count, 2'd0, underflow, f_full, f_empty, apt_fail, rct_fail, run};
            8'h03: rdata = pop_data;
            8'h04: rdata = raw_count;
            8'h05: rdata = vn_count;
            8'h06: rdata = rct_fail_count;
            8'h07: rdata = apt_fail_count;
            8'h08: rdata = word_count;
            8'h09: rdata = {26'd0, rct_cutoff};
            8'h0A: rdata = {16'd0, apt_window};
            8'h0B: rdata = {16'd0, apt_cutoff};
            8'h0C: rdata = {20'd0, temp_code};
            8'h0D: rdata = {20'd0, vccint_code};
            8'h18: rdata = {24'd0, fan_duty};
            8'h19: rdata = {16'd0, fan_rpm};
            default: rdata = 32'd0;
        endcase
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            run <= 1'b1; clear_alarms <= 1'b0;
            rct_cutoff <= 6'd20; apt_window <= 16'd1024; apt_cutoff <= 16'd660;
            fan_duty <= FAN_DUTY_DEFAULT;
            pstate <= P_IDLE; cmd_w <= 1'b0; nib <= 4'd0; sr <= 40'd0;
            tstate <= T_IDLE; txbuf <= 72'd0; txcnt <= 4'd0; t_wp <= 10'd0; t_rp <= 10'd0;
            tx_data <= 8'd0; tx_send <= 1'b0; pop <= 1'b0;
        end else begin
            tx_send <= 1'b0; clear_alarms <= 1'b0; pop <= 1'b0;

            if (txcnt != 4'd0) begin
                if (!tf_full) begin fifo_tx[t_wp[8:0]] <= txbuf[71:64]; t_wp <= t_wp + 10'd1; end
                txbuf <= {txbuf[63:0], 8'd0};
                txcnt <= txcnt - 4'd1;
            end

            if (rx_valid) begin
                case (pstate)
                    P_IDLE: begin
                        if (rx_data=="W"||rx_data=="w") begin cmd_w<=1'b1; nib<=4'd0; sr<=40'd0; pstate<=P_HEX; end
                        else if (rx_data=="R"||rx_data=="r") begin cmd_w<=1'b0; nib<=4'd0; sr<=40'd0; pstate<=P_HEX; end
                        else if (!is_eol && rx_data!=" ") pstate<=P_ERR;
                    end
                    P_HEX: begin
                        if (is_hex(rx_data)) begin sr<={sr[35:0],hex_val(rx_data)}; nib<=(nib==4'd15)?4'd15:nib+4'd1; end
                        else if (rx_data==" ") ;
                        else if (is_eol) begin
                            pstate<=P_IDLE;
                            if (cmd_w && nib==4'd10 && w_ok) begin
                                case (w_addr)
                                    8'h01: begin run<=w_data[0]; if (w_data[8]) clear_alarms<=1'b1; end
                                    8'h09: rct_cutoff<=w_data[5:0];
                                    8'h0A: apt_window<=w_data[15:0];
                                    8'h0B: apt_cutoff<=w_data[15:0];
                                    8'h18: fan_duty<=w_data[7:0];
                                endcase
                                txbuf<={"K",8'h0A,56'd0}; txcnt<=4'd2;
                            end else if (!cmd_w && nib==4'd2) begin
                                if (r_addr==8'h03) pop<=1'b1;    // reading DATA pops the FIFO
                                txbuf<={hex_chr(rdata[31:28]),hex_chr(rdata[27:24]),hex_chr(rdata[23:20]),hex_chr(rdata[19:16]),
                                        hex_chr(rdata[15:12]),hex_chr(rdata[11:8]),hex_chr(rdata[7:4]),hex_chr(rdata[3:0]),8'h0A};
                                txcnt<=4'd9;
                            end else begin txbuf<={"E",8'h0A,56'd0}; txcnt<=4'd2; end
                        end else pstate<=P_ERR;
                    end
                    default: if (is_eol) begin pstate<=P_IDLE; txbuf<={"E",8'h0A,56'd0}; txcnt<=4'd2; end
                endcase
            end

            case (tstate)
                T_IDLE: if (!tf_empty && !tx_busy) begin tx_data<=fifo_tx[t_rp[8:0]]; tx_send<=1'b1; t_rp<=t_rp+10'd1; tstate<=T_WAIT1; end
                T_WAIT1: tstate<=T_WAIT2;
                T_WAIT2: if (!tx_busy) tstate<=T_IDLE;
                default: tstate<=T_IDLE;
            endcase
        end
    end
endmodule
