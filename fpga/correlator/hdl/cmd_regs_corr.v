`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cmd_regs_corr.v -- register file for the correlator
//
// Same UART text protocol as the lockin:
//   W aa dddddddd     write register aa
//   R aa               read register aa
//
// Register map:
//   00  ID         RO  0x434F0001  ("CO" + version)
//   01  CTRL       WO  bit0: start window
//   02  SAMPLES    RW  window length (24-bit, default 10000)
//   03  STATUS     RO  bit0: busy, bits 31:16 window counter
//                      READING THIS FREEZES 04..0F
//   04  N          RO  samples in window
//   05  RAA_LO     RO  06 RAA_HI
//   07  RBB_LO     RO  08 RBB_HI
//   09  RAB_LO     RO  0A RAB_HI
//   0B  SUMA_LO    RO  0C SUMA_HI
//   0D  SUMB_LO    RO  0E SUMB_HI
//   0F  ADC_A      RO  live channel A (12-bit)
//   10  ADC_B      RO  live channel B (12-bit)
//   11  QUERY_LAG  WO  lag value (-MAX_LAG .. MAX_LAG), signed 16-bit
//   12  QUERY_GO   WO  write 1 to start lag query
//   13  XCORR_LO   RO  14 XCORR_HI   result of lag query
//   15  PPS_LAST   RO  GPS PPS sample counter
//   16  FAN_RPM    RO  fan speed
//-----------------------------------------------------------------------------
module cmd_regs_corr #(
    parameter integer CLKS_PER_BIT = 434
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx,
    output wire               tx,

    output reg                ctrl_start,
    output reg  [23:0]        win_samples,
    output reg  signed [15:0] query_lag,
    output reg                query_go,

    input  wire               busy,
    input  wire               done,
    input  wire signed [63:0] raa,
    input  wire signed [63:0] rbb,
    input  wire signed [63:0] rab,
    input  wire signed [63:0] sum_a,
    input  wire signed [63:0] sum_b,
    input  wire [31:0]        n_out,
    input  wire signed [63:0] xcorr_val,
    input  wire               xcorr_valid,
    input  wire [11:0]        adc_a,
    input  wire [11:0]        adc_b,
    input  wire [31:0]        pps_last,
    input  wire [15:0]        fan_rpm
);
    localparam [31:0] ID_WORD = 32'h434F0001;

    // UART
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_send;
    wire       tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_data), .valid(rx_valid));
    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_data), .send(tx_send), .tx(tx), .busy(tx_busy));

    // helpers
    function is_hex(input [7:0] c);
        is_hex = ((c >= "0") && (c <= "9")) || ((c >= "A") && (c <= "F")) || ((c >= "a") && (c <= "f"));
    endfunction
    function [3:0] hex_val(input [7:0] c);
        if (c <= "9") hex_val = c[3:0]; else hex_val = c[3:0] + 4'd9;
    endfunction
    function [7:0] hex_chr(input [3:0] n);
        hex_chr = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});
    endfunction

    // snapshot registers (frozen on STATUS read)
    reg [15:0]        seq = 0;
    reg signed [63:0] sh_raa, sh_rbb, sh_rab;
    reg signed [63:0] sh_sa, sh_sb;
    reg [31:0]        sh_n;
    reg signed [63:0] sh_xcorr;

    // parser state
    reg [1:0]  pstate;
    localparam P_IDLE = 2'd0, P_HEX = 2'd1, P_ERR = 2'd2;
    reg        cmd_w;
    reg [3:0]  nib;
    reg [39:0] sr;

    // transmitter state
    reg [1:0]  tstate;
    localparam T_IDLE = 2'd0, T_LOAD = 2'd1, T_WAIT1 = 2'd2, T_WAIT2 = 2'd3;
    reg [71:0] txbuf;
    reg [3:0]  txcnt;

    wire [7:0]  w_addr = sr[39:32];
    wire [31:0] w_data = sr[31:0];
    wire [7:0]  r_addr = sr[7:0];

    // read mux
    reg [31:0] rdata;
    always @(*) begin
        case (r_addr)
            8'h00: rdata = ID_WORD;
            8'h02: rdata = {8'd0, win_samples};
            8'h03: rdata = {seq, 15'd0, busy};
            8'h04: rdata = sh_n;
            8'h05: rdata = sh_raa[31:0];
            8'h06: rdata = sh_raa[63:32];
            8'h07: rdata = sh_rbb[31:0];
            8'h08: rdata = sh_rbb[63:32];
            8'h09: rdata = sh_rab[31:0];
            8'h0A: rdata = sh_rab[63:32];
            8'h0B: rdata = sh_sa[31:0];
            8'h0C: rdata = sh_sa[63:32];
            8'h0D: rdata = sh_sb[31:0];
            8'h0E: rdata = sh_sb[63:32];
            8'h0F: rdata = {20'd0, adc_a};
            8'h10: rdata = {20'd0, adc_b};
            8'h11: rdata = {{16{query_lag[15]}}, query_lag};
            8'h13: rdata = sh_xcorr[31:0];
            8'h14: rdata = sh_xcorr[63:32];
            8'h15: rdata = pps_last;
            8'h16: rdata = {16'd0, fan_rpm};
            default: rdata = 32'd0;
        endcase
    end

    wire is_eol = (rx_data == 8'h0A) || (rx_data == 8'h0D);

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl_start  <= 1'b0;
            win_samples <= 24'd10000;
            query_lag   <= 16'sd0;
            query_go    <= 1'b0;
            seq         <= 16'd0;
            pstate      <= P_IDLE;
            cmd_w       <= 1'b0;
            nib         <= 4'd0;
            sr          <= 40'd0;
            tstate      <= T_IDLE;
            txbuf       <= 72'd0;
            txcnt       <= 4'd0;
            tx_data     <= 8'd0;
            tx_send     <= 1'b0;
            sh_raa <= 0; sh_rbb <= 0; sh_rab <= 0;
            sh_sa  <= 0; sh_sb  <= 0; sh_n   <= 0;
            sh_xcorr <= 0;
        end else begin
            ctrl_start <= 1'b0;
            query_go   <= 1'b0;
            tx_send    <= 1'b0;

            if (done) seq <= seq + 16'd1;
            if (xcorr_valid) sh_xcorr <= xcorr_val;

            // parser
            if (rx_valid) begin
                case (pstate)
                    P_IDLE: begin
                        if (rx_data == "W" || rx_data == "w") begin
                            cmd_w <= 1'b1; nib <= 0; sr <= 0; pstate <= P_HEX;
                        end else if (rx_data == "R" || rx_data == "r") begin
                            cmd_w <= 1'b0; nib <= 0; sr <= 0; pstate <= P_HEX;
                        end else if (!is_eol && rx_data != " ") pstate <= P_ERR;
                    end
                    P_HEX: begin
                        if (is_hex(rx_data)) begin
                            sr  <= {sr[35:0], hex_val(rx_data)};
                            nib <= (nib == 4'd15) ? 4'd15 : nib + 4'd1;
                        end else if (rx_data == " ") begin
                            // ignore
                        end else if (is_eol) begin
                            pstate <= P_IDLE;
                            if (cmd_w && nib == 4'd10) begin
                                case (w_addr)
                                    8'h01: ctrl_start  <= w_data[0];
                                    8'h02: win_samples <= w_data[23:0];
                                    8'h11: query_lag   <= w_data[15:0];
                                    8'h12: query_go    <= w_data[0];
                                    default: ;
                                endcase
                                txbuf <= {"K", 8'h0A, 56'd0}; txcnt <= 4'd2; tstate <= T_LOAD;
                            end else if (!cmd_w && nib == 4'd2) begin
                                if (r_addr == 8'h03) begin
                                    sh_raa <= raa; sh_rbb <= rbb; sh_rab <= rab;
                                    sh_sa  <= sum_a; sh_sb  <= sum_b; sh_n  <= n_out;
                                end
                                txbuf <= {hex_chr(rdata[31:28]), hex_chr(rdata[27:24]),
                                          hex_chr(rdata[23:20]), hex_chr(rdata[19:16]),
                                          hex_chr(rdata[15:12]), hex_chr(rdata[11:8]),
                                          hex_chr(rdata[7:4]),   hex_chr(rdata[3:0]), 8'h0A};
                                txcnt <= 4'd9; tstate <= T_LOAD;
                            end else begin
                                txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 4'd2; tstate <= T_LOAD;
                            end
                        end else pstate <= P_ERR;
                    end
                    default: begin
                        if (is_eol) begin
                            pstate <= P_IDLE;
                            txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 4'd2; tstate <= T_LOAD;
                        end
                    end
                endcase
            end

            // transmitter
            case (tstate)
                T_LOAD: begin
                    if (txcnt == 0) tstate <= T_IDLE;
                    else if (!tx_busy) begin
                        tx_data <= txbuf[71:64]; tx_send <= 1'b1; tstate <= T_WAIT1;
                    end
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
