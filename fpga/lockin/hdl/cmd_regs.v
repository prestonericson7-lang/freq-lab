`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cmd_regs.v -- register file driven by text commands over a UART
//
// Protocol (ASCII, 115200 8N1, one command per line, spaces ignored):
//     W aa dddddddd      write 32-bit hex value to register aa   -> reply "K"
//     R aa               read register aa                        -> reply 8 hex digits
//     anything else                                              -> reply "E"
//
// Register map
//   00  ID          RO  0x4C4B0001
//   01  CTRL        RW  bit0 DAC on, bit1 continuous windows, bit2 resonance
//                       tracking on, bit3 invert tracking direction, bit4 sync pin on
//   02  FTW_LO      RW  tuning word, low 32 bits   (frequency = FTW * 50e6 / 2^48)
//   03  FTW_HI      RW  tuning word, high 16 bits; writing this loads the pair
//   04  AMP         RW  drive amplitude 0 .. 58982 (= 90 % of full scale)
//   05  CYCLES      RW  drive cycles per lock-in window (24 bit)
//   06  DC_SHIFT    RW  DC tracker time constant = 2^n samples (0..31)
//   07  START       WO  write 1: measure one window
//   08  STATUS      RO  bit0 busy, bit1 ADC alive, bits 23:8 window counter.
//                       READING THIS FREEZES registers 09..16 so that a set of
//                       reads belongs to one window.
//   09  N           RO  samples in the window
//   0A  I_LO   0B I_HI   0C Q_LO   0D Q_HI      RO  64-bit signed sums
//   0E  SUMX_LO 0F SUMX_HI                      RO  sum of raw ADC codes (48 bit)
//   10  ADC_RAW     RO  latest raw ADC code (live)
//   11  DC_EST      RO  DC tracker value in ADC counts (live)
//   12  TRACK_STEP  RW  tuning-word step applied per window while tracking
//   13  TRACK_COS   RW  cos(target lag) * 32767, signed 16 bit
//   14  TRACK_SIN   RW  sin(target lag) * 32767, signed 16 bit
//   15  FTW_NOW_LO  RO  tuning word in use (moves while tracking)
//   16  FTW_NOW_HI  RO
//-----------------------------------------------------------------------------
module cmd_regs #(
    parameter integer CLKS_PER_BIT = 434
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               rx,
    output wire               tx,

    output reg  [4:0]         ctrl,
    output reg  [47:0]        ftw_set,
    output reg                ftw_load,
    output reg  [15:0]        amp,
    output reg  [23:0]        cycles,
    output reg  [4:0]         dc_shift,
    output reg                start,
    output reg  [31:0]        track_step,
    output reg  signed [15:0] track_cos,
    output reg  signed [15:0] track_sin,

    input  wire               busy,
    input  wire               adc_seen,
    input  wire               done,
    input  wire signed [63:0] res_i,
    input  wire signed [63:0] res_q,
    input  wire [31:0]        res_n,
    input  wire [47:0]        res_sumx,
    input  wire [11:0]        adc_last,
    input  wire [11:0]        dc_est,
    input  wire [47:0]        ftw_now
);
    localparam [31:0] ID_WORD = 32'h4C4B0001;
    localparam [15:0] AMP_MAX = 16'd58982;

    // ------------------------------------------------------------------ UART
    wire [7:0] rx_data;
    wire       rx_valid;
    reg  [7:0] tx_data;
    reg        tx_send;
    wire       tx_busy;

    uart_rx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(rx), .data(rx_data), .valid(rx_valid));

    uart_tx #(.CLKS_PER_BIT(CLKS_PER_BIT)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_data), .send(tx_send), .tx(tx), .busy(tx_busy));

    // ------------------------------------------------------------------ helpers
    function is_hex(input [7:0] c);
        is_hex = ((c >= "0") && (c <= "9")) || ((c >= "A") && (c <= "F")) || ((c >= "a") && (c <= "f"));
    endfunction

    function [3:0] hex_val(input [7:0] c);
        if (c <= "9")      hex_val = c[3:0];            // '0'..'9' = 0x30..0x39
        else               hex_val = c[3:0] + 4'd9;     // 'A'/'a' = 0x41/0x61 -> 1 + 9
    endfunction

    function [7:0] hex_chr(input [3:0] n);
        hex_chr = (n < 4'd10) ? (8'h30 + {4'd0, n}) : (8'h37 + {4'd0, n});   // 0-9, A-F
    endfunction

    // ------------------------------------------------------------------ state
    reg [15:0] seq;
    reg [31:0] ftw_lo_hold;

    reg signed [63:0] sh_i, sh_q;
    reg [31:0]        sh_n;
    reg [47:0]        sh_sumx;
    reg [47:0]        sh_ftw;

    localparam [1:0] P_IDLE = 2'd0, P_HEX = 2'd1, P_ERR = 2'd2;
    reg [1:0]  pstate;
    reg        cmd_w;
    reg [3:0]  nib;
    reg [39:0] sr;

    localparam [1:0] T_IDLE = 2'd0, T_LOAD = 2'd1, T_WAIT1 = 2'd2, T_WAIT2 = 2'd3;
    reg [1:0]  tstate;
    reg [71:0] txbuf;
    reg [3:0]  txcnt;

    wire [7:0]  w_addr = sr[39:32];
    wire [31:0] w_data = sr[31:0];
    wire [7:0]  r_addr = sr[7:0];

    reg [31:0] rdata;
    always @(*) begin
        case (r_addr)
            8'h00:   rdata = ID_WORD;
            8'h01:   rdata = {27'd0, ctrl};
            8'h02:   rdata = ftw_set[31:0];
            8'h03:   rdata = {16'd0, ftw_set[47:32]};
            8'h04:   rdata = {16'd0, amp};
            8'h05:   rdata = {8'd0, cycles};
            8'h06:   rdata = {27'd0, dc_shift};
            8'h08:   rdata = {8'd0, seq, 6'd0, adc_seen, busy};
            8'h09:   rdata = sh_n;
            8'h0A:   rdata = sh_i[31:0];
            8'h0B:   rdata = sh_i[63:32];
            8'h0C:   rdata = sh_q[31:0];
            8'h0D:   rdata = sh_q[63:32];
            8'h0E:   rdata = sh_sumx[31:0];
            8'h0F:   rdata = {16'd0, sh_sumx[47:32]};
            8'h10:   rdata = {20'd0, adc_last};
            8'h11:   rdata = {20'd0, dc_est};
            8'h12:   rdata = track_step;
            8'h13:   rdata = {{16{track_cos[15]}}, track_cos};
            8'h14:   rdata = {{16{track_sin[15]}}, track_sin};
            8'h15:   rdata = sh_ftw[31:0];
            8'h16:   rdata = {16'd0, sh_ftw[47:32]};
            default: rdata = 32'd0;
        endcase
    end

    wire is_eol = (rx_data == 8'h0A) || (rx_data == 8'h0D);

    always @(posedge clk) begin
        if (!rst_n) begin
            ctrl        <= 5'd0;
            ftw_set     <= 48'd0;
            ftw_load    <= 1'b0;
            amp         <= 16'd0;
            cycles      <= 24'd10;
            dc_shift    <= 5'd18;
            start       <= 1'b0;
            track_step  <= 32'd0;
            track_cos   <= 16'sd32767;
            track_sin   <= 16'sd0;
            seq         <= 16'd0;
            ftw_lo_hold <= 32'd0;
            sh_i        <= 64'sd0;
            sh_q        <= 64'sd0;
            sh_n        <= 32'd0;
            sh_sumx     <= 48'd0;
            sh_ftw      <= 48'd0;
            pstate      <= P_IDLE;
            cmd_w       <= 1'b0;
            nib         <= 4'd0;
            sr          <= 40'd0;
            tstate      <= T_IDLE;
            txbuf       <= 72'd0;
            txcnt       <= 4'd0;
            tx_data     <= 8'd0;
            tx_send     <= 1'b0;
        end else begin
            ftw_load <= 1'b0;
            start    <= 1'b0;
            tx_send  <= 1'b0;

            if (done) seq <= seq + 16'd1;

            // ---------------------------------------------------- command parser
            if (rx_valid) begin
                case (pstate)
                    P_IDLE: begin
                        if (rx_data == "W" || rx_data == "w") begin
                            cmd_w <= 1'b1; nib <= 4'd0; sr <= 40'd0; pstate <= P_HEX;
                        end else if (rx_data == "R" || rx_data == "r") begin
                            cmd_w <= 1'b0; nib <= 4'd0; sr <= 40'd0; pstate <= P_HEX;
                        end else if (!is_eol && rx_data != " ") begin
                            pstate <= P_ERR;
                        end
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
                                    8'h01: ctrl        <= w_data[4:0];
                                    8'h02: ftw_lo_hold <= w_data;
                                    8'h03: begin
                                        ftw_set  <= {w_data[15:0], ftw_lo_hold};
                                        ftw_load <= 1'b1;
                                    end
                                    8'h04: amp        <= (w_data[15:0] > AMP_MAX || w_data[31:16] != 16'd0)
                                                         ? AMP_MAX : w_data[15:0];
                                    8'h05: cycles     <= w_data[23:0];
                                    8'h06: dc_shift   <= w_data[4:0];
                                    8'h07: start      <= w_data[0];
                                    8'h12: track_step <= w_data;
                                    8'h13: track_cos  <= w_data[15:0];
                                    8'h14: track_sin  <= w_data[15:0];
                                    default: ;
                                endcase
                                txbuf  <= {"K", 8'h0A, 56'd0};
                                txcnt  <= 4'd2;
                                tstate <= T_LOAD;
                            end else if (!cmd_w && nib == 4'd2) begin
                                if (r_addr == 8'h08) begin          // freeze one window's results
                                    sh_i    <= res_i;
                                    sh_q    <= res_q;
                                    sh_n    <= res_n;
                                    sh_sumx <= res_sumx;
                                    sh_ftw  <= ftw_now;
                                end
                                txbuf  <= {hex_chr(rdata[31:28]), hex_chr(rdata[27:24]),
                                           hex_chr(rdata[23:20]), hex_chr(rdata[19:16]),
                                           hex_chr(rdata[15:12]), hex_chr(rdata[11:8]),
                                           hex_chr(rdata[7:4]),   hex_chr(rdata[3:0]), 8'h0A};
                                txcnt  <= 4'd9;
                                tstate <= T_LOAD;
                            end else begin
                                txbuf  <= {"E", 8'h0A, 56'd0};
                                txcnt  <= 4'd2;
                                tstate <= T_LOAD;
                            end
                        end else begin
                            pstate <= P_ERR;
                        end
                    end

                    default: begin                       // P_ERR: swallow the rest of the line
                        if (is_eol) begin
                            pstate <= P_IDLE;
                            txbuf  <= {"E", 8'h0A, 56'd0};
                            txcnt  <= 4'd2;
                            tstate <= T_LOAD;
                        end
                    end
                endcase
            end

            // ---------------------------------------------------- reply sender
            case (tstate)
                T_LOAD: begin
                    if (txcnt == 4'd0) begin
                        tstate <= T_IDLE;
                    end else if (!tx_busy) begin
                        tx_data <= txbuf[71:64];
                        tx_send <= 1'b1;
                        tstate  <= T_WAIT1;
                    end
                end
                T_WAIT1: tstate <= T_WAIT2;               // let the transmitter raise busy
                T_WAIT2: begin
                    if (!tx_busy) begin
                        txbuf  <= {txbuf[63:0], 8'd0};
                        txcnt  <= txcnt - 4'd1;
                        tstate <= T_LOAD;
                    end
                end
                default: ;
            endcase
        end
    end

endmodule
