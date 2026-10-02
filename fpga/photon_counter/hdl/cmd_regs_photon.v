`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cmd_regs_photon.v -- register file for the photon counter
//
// Register map:
//   00  ID           RO  0x50480001  ("PH" + version)
//   01  CTRL         WO  bit0: start, bit1: stop
//   02  WINDOW       RW  coincidence window in clock ticks (16-bit, default 5
//                        = 100 ns at 50 MHz).  1 tick = 20 ns.
//   03  STATUS       RO  bit0: busy.  READING FREEZES 04..09.
//   04  SINGLES_A_LO RO  05 SINGLES_A_HI  (48-bit)
//   06  SINGLES_B_LO RO  07 SINGLES_B_HI  (48-bit)
//   08  COINC_LO     RO  09 COINC_HI      (48-bit)
//   0A  ELAPSED      RO  clock ticks since start (32-bit)
//   0B  FAN_RPM      RO
//-----------------------------------------------------------------------------
module cmd_regs_photon #(
    parameter integer CLKS_PER_BIT = 434
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        rx,
    output wire        tx,

    output reg         coinc_start,
    output reg         coinc_stop,
    output reg  [15:0] coinc_window,

    input  wire        busy,
    input  wire        done,
    input  wire [47:0] singles_a,
    input  wire [47:0] singles_b,
    input  wire [47:0] coincidences,
    input  wire [31:0] elapsed_clk,
    input  wire [15:0] fan_rpm
);
    localparam [31:0] ID_WORD = 32'h50480001;

    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_send;
    wire       tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_data), .valid(rx_valid));
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_data), .send(tx_send), .tx(tx), .busy(tx_busy));

    function is_hex(input [7:0] c);
        is_hex = ((c >= "0") && (c <= "9")) || ((c >= "A") && (c <= "F")) || ((c >= "a") && (c <= "f"));
    endfunction
    function [3:0] hex_val(input [7:0] c);
        if (c <= "9") hex_val = c[3:0]; else hex_val = c[3:0] + 4'd9;
    endfunction
    function [7:0] hex_chr(input [3:0] n);
        hex_chr = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});
    endfunction

    // snapshot
    reg [47:0] sh_sa, sh_sb, sh_co;
    reg [31:0] sh_el;
    reg [15:0] seq = 0;

    // parser
    reg [1:0]  pstate;
    localparam P_IDLE = 2'd0, P_HEX = 2'd1, P_ERR = 2'd2;
    reg cmd_w;
    reg [3:0] nib;
    reg [39:0] sr;

    // transmitter
    reg [1:0]  tstate;
    localparam T_IDLE = 2'd0, T_LOAD = 2'd1, T_WAIT1 = 2'd2, T_WAIT2 = 2'd3;
    reg [71:0] txbuf;
    reg [3:0]  txcnt;

    wire [7:0]  w_addr = sr[39:32];
    wire [31:0] w_data = sr[31:0];
    wire [7:0]  r_addr = sr[7:0];

    reg [31:0] rdata;
    always @(*) begin
        case (r_addr)
            8'h00: rdata = ID_WORD;
            8'h02: rdata = {16'd0, coinc_window};
            8'h03: rdata = {seq, 15'd0, busy};
            8'h04: rdata = sh_sa[31:0];
            8'h05: rdata = {16'd0, sh_sa[47:32]};
            8'h06: rdata = sh_sb[31:0];
            8'h07: rdata = {16'd0, sh_sb[47:32]};
            8'h08: rdata = sh_co[31:0];
            8'h09: rdata = {16'd0, sh_co[47:32]};
            8'h0A: rdata = sh_el;
            8'h0B: rdata = {16'd0, fan_rpm};
            default: rdata = 32'd0;
        endcase
    end

    wire is_eol = (rx_data == 8'h0A) || (rx_data == 8'h0D);

    always @(posedge clk) begin
        if (!rst_n) begin
            coinc_start  <= 0; coinc_stop <= 0;
            coinc_window <= 16'd5;
            seq <= 0;
            pstate <= P_IDLE; cmd_w <= 0; nib <= 0; sr <= 0;
            tstate <= T_IDLE; txbuf <= 0; txcnt <= 0;
            tx_data <= 0; tx_send <= 0;
            sh_sa <= 0; sh_sb <= 0; sh_co <= 0; sh_el <= 0;
        end else begin
            coinc_start <= 0; coinc_stop <= 0; tx_send <= 0;
            if (done) seq <= seq + 1;

            if (rx_valid) begin
                case (pstate)
                    P_IDLE: begin
                        if (rx_data == "W" || rx_data == "w") begin
                            cmd_w <= 1; nib <= 0; sr <= 0; pstate <= P_HEX;
                        end else if (rx_data == "R" || rx_data == "r") begin
                            cmd_w <= 0; nib <= 0; sr <= 0; pstate <= P_HEX;
                        end else if (!is_eol && rx_data != " ") pstate <= P_ERR;
                    end
                    P_HEX: begin
                        if (is_hex(rx_data)) begin
                            sr <= {sr[35:0], hex_val(rx_data)};
                            nib <= (nib == 4'd15) ? 4'd15 : nib + 1;
                        end else if (rx_data == " ") begin
                        end else if (is_eol) begin
                            pstate <= P_IDLE;
                            if (cmd_w && nib == 4'd10) begin
                                case (w_addr)
                                    8'h01: begin
                                        coinc_start <= w_data[0];
                                        coinc_stop  <= w_data[1];
                                    end
                                    8'h02: coinc_window <= w_data[15:0];
                                    default: ;
                                endcase
                                txbuf <= {"K", 8'h0A, 56'd0}; txcnt <= 2; tstate <= T_LOAD;
                            end else if (!cmd_w && nib == 4'd2) begin
                                if (r_addr == 8'h03) begin
                                    sh_sa <= singles_a; sh_sb <= singles_b;
                                    sh_co <= coincidences; sh_el <= elapsed_clk;
                                end
                                txbuf <= {hex_chr(rdata[31:28]), hex_chr(rdata[27:24]),
                                          hex_chr(rdata[23:20]), hex_chr(rdata[19:16]),
                                          hex_chr(rdata[15:12]), hex_chr(rdata[11:8]),
                                          hex_chr(rdata[7:4]),   hex_chr(rdata[3:0]), 8'h0A};
                                txcnt <= 9; tstate <= T_LOAD;
                            end else begin
                                txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 2; tstate <= T_LOAD;
                            end
                        end else pstate <= P_ERR;
                    end
                    default: if (is_eol) begin
                        pstate <= P_IDLE;
                        txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 2; tstate <= T_LOAD;
                    end
                endcase
            end

            case (tstate)
                T_LOAD: if (txcnt == 0) tstate <= T_IDLE;
                        else if (!tx_busy) begin
                            tx_data <= txbuf[71:64]; tx_send <= 1; tstate <= T_WAIT1;
                        end
                T_WAIT1: tstate <= T_WAIT2;
                T_WAIT2: if (!tx_busy) begin
                    txbuf <= {txbuf[63:0], 8'd0}; txcnt <= txcnt - 1; tstate <= T_LOAD;
                end
                default: ;
            endcase
        end
    end

endmodule
