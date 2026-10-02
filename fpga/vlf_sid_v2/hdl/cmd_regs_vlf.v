`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cmd_regs_vlf.v -- UART register file for the VLF SID receiver
//
// Same text protocol as the other freq-lab designs (115200 8N1):
//   W aa dddddddd\n    write register aa  -> "K\n"  ("E\n" if aa is not writable)
//   R aa\n             read register aa   -> "dddddddd\n"
//   anything else                          -> "E\n"
// Replies go through a 512-byte FIFO, so a host may send a whole batch of
// commands without waiting (one USB round trip for a full snapshot); the
// replies come back in order.
//
// Register map:
//   00  ID       RO  0x564C0001 ("VL", version 1)
//   01  CTRL     RW  bit0 RUN (1), bit1 PPS_MODE (1)            default 0x3
//   02  WIN      RW  window length in samples (24 bit)         default 390625 (1 s)
//   03  STATUS   RO  31:16 window count (live, low 16 bits)
//                    bit3 PPS alive, bit2 last window ended on timeout,
//                    bit1 last window ended on a PPS edge, bit0 RUN
//                    READING STATUS COPIES THE LAST FINISHED WINDOW INTO 04..0D
//                    AND 20..3F, so a set of reads after it is always consistent.
//   04  SEQ      RO  window number of that copy
//   05  NSAMP    RO  samples in the window (= fs over a GPS second)
//   06  NDEC     RO  decimated outputs per channel in the window (~381 per s)
//   07  FLAGS    RO  bit0 ended on PPS, bit1 started on PPS, bit2 ended on timeout
//   08  CLIP     RO  samples at code 0 or 4095 (input overload)
//   09  XMINMAX  RO  {4'b0, max code, 4'b0, min code}
//   0A  SQ_LO    RO  sum of (code-2048)^2, low word   (input power)
//   0B  SQ_HI    RO  high word
//   0C  SUM_LO   RO  sum of (code-2048), low word     (signed 64-bit, DC offset)
//   0D  SUM_HI   RO  high word
//   0E  ADC      RO  live ADC code
//   0F  PPS_CNT  RO  PPS edges since power-up (live)
//   10..17 FTW0..7  RW  channel tuning words, f = FTW * fs / 2^32
//   18  FAN_DUTY RW  0..100 %                                   default 60
//   19  FAN_RPM  RO
//   1A  FS_CLKS  RO  50 MHz clocks per 65536 ADC samples (8388608 = 390625 S/s);
//                    fs = 65536 * 50e6 / FS_CLKS. 0 for the first 0.17 s.
//   20+4k  POW_LO(k)  RO  channel k: sum of I^2+Q^2 over the window, low word
//   21+4k  POW_HI(k)  RO  high word
//   22+4k  PMAX(k)    RO  largest single I^2+Q^2 in the window, bits 47:16
//   23+4k  -          RO  0
//   amplitude (ADC counts) = sqrt(POW / NDEC) / 2047.9375
//
// Lightning (sferic) capture, version 2:
//   40  SF_THRESH  RW  |code-2048| at or above this triggers an event; 0 = off  default 0
//   41  SF_HOLDOFF RW  samples ignored after a trigger                        default 1953 (5 ms)
//   42  SF_COUNT   RO  events captured since power-up
//   43  SF_NEV     RO  events waiting in the FIFO (0..32)
//   44  SF_POP     RO  reading it drops the oldest event; returns SF_NEV before the drop
//   45  EV_PPS     RO  oldest event: PPS_CNT at the trigger (which GPS second)
//   46  EV_CLK     RO  bit 31 = a PPS had been seen; [25:0] 50 MHz clocks since that PPS (20 ns)
//   47  EV_INFO    RO  {pre-trigger samples (16), snapshot length (64), 3'b0, polarity, |peak|}
//   48  EV_SEQ     RO  event number
//   49  SF_LOST    RO  triggers dropped because the FIFO was full
//   60..7F EV_SNAP RO  oldest event's 64 raw ADC codes, two per word: {4'b0, s[2k+1], 4'b0, s[2k]}
//                      s[0..15] precede the trigger, s[16] is the trigger sample
//-----------------------------------------------------------------------------
module cmd_regs_vlf #(
    parameter integer CLKS_PER_BIT     = 434,
    parameter integer FAN_DUTY_DEFAULT = 60
)(
    input  wire          clk,
    input  wire          rst_n,
    input  wire          rx,
    output wire          tx,

    output reg           run,
    output reg           pps_mode,
    output reg  [23:0]   win_len,
    output reg  [255:0]  ftw_bus,
    output reg  [7:0]    fan_duty,

    input  wire [31:0]   seq,
    input  wire          pps_alive,
    input  wire          last_end_pps,
    input  wire          last_end_to,
    input  wire [31:0]   w_n,
    input  wire [31:0]   w_ndec,
    input  wire [31:0]   w_clip,
    input  wire [2:0]    w_flags,
    input  wire [11:0]   w_min,
    input  wire [11:0]   w_max,
    input  wire [63:0]   w_sq,
    input  wire [63:0]   w_sum,
    input  wire [511:0]  w_pow_bus,
    input  wire [383:0]  w_pmax_bus,
    input  wire [11:0]   adc_live,
    input  wire [31:0]   pps_cnt,
    input  wire [15:0]   fan_rpm,
    input  wire [31:0]   fs_clks,

    output reg  [11:0]   sf_thresh,
    output reg  [15:0]   sf_holdoff,
    output reg           sf_ack,
    input  wire [31:0]   sf_count,
    input  wire [31:0]   sf_lost,
    input  wire [5:0]    sf_nev,
    input  wire [31:0]   ev_pps,
    input  wire [31:0]   ev_clk,
    input  wire [31:0]   ev_info,
    input  wire [31:0]   ev_seq,
    output wire [4:0]    sf_snap_addr,
    input  wire [31:0]   sf_snap_data
);
    localparam [31:0] ID_WORD = 32'h564C0002;

    // default tuning words at fs = 390625 S/s (sim/vlf_model.py prints these)
    localparam [31:0] FTW0_DEF = 32'h1040BFE4;   // NLK 24.80 kHz Jim Creek WA
    localparam [31:0] FTW1_DEF = 32'h0E065300;   // NPM 21.40 kHz Lualualei HI
    localparam [31:0] FTW2_DEF = 32'h0FBA8827;   // NAA 24.00 kHz Cutler ME
    localparam [31:0] FTW3_DEF = 32'h1083DBC2;   // NML 25.20 kHz LaMoure ND
    localparam [31:0] FTW4_DEF = 32'h1AB4B72C;   // NAU 40.75 kHz Aguada PR
    localparam [31:0] FTW5_DEF = 32'h0CF9E386;   // NWC 19.80 kHz Exmouth, Australia
    localparam [31:0] FTW6_DEF = 32'h0E8C8ABD;   // JJI 22.20 kHz Ebino, Japan
    localparam [31:0] FTW7_DEF = 32'h13A92A30;   // 30.00 kHz, no station (noise reference)

    // ------------------------------------------------ UART
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

    // ------------------------------------------------ snapshot (copied on STATUS read)
    reg [31:0]  sh_seq, sh_n, sh_ndec, sh_clip;
    reg [2:0]   sh_flags;
    reg [11:0]  sh_min, sh_max;
    reg [63:0]  sh_sq, sh_sum;
    reg [511:0] sh_pow;
    reg [383:0] sh_pmax;

    // ------------------------------------------------ parser / transmitter state
    reg [1:0]  pstate;
    localparam P_IDLE = 2'd0, P_HEX = 2'd1, P_ERR = 2'd2;
    reg        cmd_w;
    reg [3:0]  nib;
    reg [39:0] sr;

    // reply loader: a finished reply (txbuf, txcnt bytes) is copied into the
    // FIFO one byte per clock -- 9 clocks, far less than one UART byte time
    reg [71:0] txbuf;
    reg [3:0]  txcnt;

    // reply FIFO, 512 bytes (distributed RAM)
    reg [7:0]  fifo [0:511];
    reg [9:0]  f_wp, f_rp;
    wire       f_empty = (f_wp == f_rp);
    wire       f_full  = (f_wp[8:0] == f_rp[8:0]) && (f_wp[9] != f_rp[9]);

    always @(posedge clk) begin
        if (rst_n && txcnt != 4'd0 && !f_full)
            fifo[f_wp[8:0]] <= txbuf[71:64];
    end

    // transmitter: FIFO -> uart_tx
    reg [1:0]  tstate;
    localparam T_IDLE = 2'd0, T_WAIT1 = 2'd1, T_WAIT2 = 2'd2;

    wire [7:0]  w_addr = sr[39:32];
    wire [31:0] w_data = sr[31:0];
    wire [7:0]  r_addr = sr[7:0];

    wire        w_ok = (w_addr == 8'h01) || (w_addr == 8'h02) ||
                       (w_addr[7:3] == 5'b00010) || (w_addr == 8'h18) ||
                       (w_addr == 8'h40) || (w_addr == 8'h41);

    assign sf_snap_addr = r_addr[4:0];          // the word of a 0x60..0x7F read, stable before EOL

    // ------------------------------------------------ read mux
    wire [2:0]  rk = r_addr[4:2];               // channel of a 0x20..0x3F read
    reg  [31:0] rdata;
    always @(*) begin
        rdata = 32'd0;
        if (r_addr[7:5] == 3'b011) begin
            rdata = sf_snap_data;
        end else if (r_addr[7:5] == 3'b001) begin
            case (r_addr[1:0])
                2'd0: rdata = sh_pow[rk*64 +: 32];
                2'd1: rdata = sh_pow[rk*64 + 32 +: 32];
                2'd2: rdata = sh_pmax[rk*48 + 16 +: 32];
                default: rdata = 32'd0;
            endcase
        end else if (r_addr[7:3] == 5'b00010) begin
            rdata = ftw_bus[r_addr[2:0]*32 +: 32];
        end else begin
            case (r_addr)
                8'h00: rdata = ID_WORD;
                8'h01: rdata = {30'd0, pps_mode, run};
                8'h02: rdata = {8'd0, win_len};
                8'h03: rdata = {seq[15:0], 12'd0, pps_alive, last_end_to, last_end_pps, run};
                8'h04: rdata = sh_seq;
                8'h05: rdata = sh_n;
                8'h06: rdata = sh_ndec;
                8'h07: rdata = {29'd0, sh_flags};
                8'h08: rdata = sh_clip;
                8'h09: rdata = {4'd0, sh_max, 4'd0, sh_min};
                8'h0A: rdata = sh_sq[31:0];
                8'h0B: rdata = sh_sq[63:32];
                8'h0C: rdata = sh_sum[31:0];
                8'h0D: rdata = sh_sum[63:32];
                8'h0E: rdata = {20'd0, adc_live};
                8'h0F: rdata = pps_cnt;
                8'h18: rdata = {24'd0, fan_duty};
                8'h19: rdata = {16'd0, fan_rpm};
                8'h1A: rdata = fs_clks;
                8'h40: rdata = {20'd0, sf_thresh};
                8'h41: rdata = {16'd0, sf_holdoff};
                8'h42: rdata = sf_count;
                8'h43: rdata = {26'd0, sf_nev};
                8'h44: rdata = {26'd0, sf_nev};
                8'h45: rdata = ev_pps;
                8'h46: rdata = ev_clk;
                8'h47: rdata = ev_info;
                8'h48: rdata = ev_seq;
                8'h49: rdata = sf_lost;
                default: rdata = 32'd0;
            endcase
        end
    end

    wire is_eol = (rx_data == 8'h0A) || (rx_data == 8'h0D);

    always @(posedge clk) begin
        if (!rst_n) begin
            run      <= 1'b1;
            pps_mode <= 1'b1;
            win_len  <= 24'd390625;
            ftw_bus  <= {FTW7_DEF, FTW6_DEF, FTW5_DEF, FTW4_DEF,
                         FTW3_DEF, FTW2_DEF, FTW1_DEF, FTW0_DEF};
            fan_duty <= FAN_DUTY_DEFAULT;
            sf_thresh  <= 12'd0;
            sf_holdoff <= 16'd1953;
            sf_ack     <= 1'b0;
            pstate   <= P_IDLE;
            cmd_w    <= 1'b0;
            nib      <= 4'd0;
            sr       <= 40'd0;
            tstate   <= T_IDLE;
            txbuf    <= 72'd0;
            txcnt    <= 4'd0;
            f_wp     <= 10'd0;
            f_rp     <= 10'd0;
            tx_data  <= 8'd0;
            tx_send  <= 1'b0;
            sh_seq <= 0; sh_n <= 0; sh_ndec <= 0; sh_clip <= 0; sh_flags <= 0;
            sh_min <= 0; sh_max <= 0; sh_sq <= 0; sh_sum <= 0;
            sh_pow <= 0; sh_pmax <= 0;
        end else begin
            tx_send <= 1'b0;
            sf_ack  <= 1'b0;

            // ---------------- reply loader (txbuf -> FIFO); a full FIFO drops bytes
            if (txcnt != 4'd0) begin
                if (!f_full) f_wp <= f_wp + 10'd1;
                txbuf <= {txbuf[63:0], 8'd0};
                txcnt <= txcnt - 4'd1;
            end

            // ---------------- parser
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
                            // separators are ignored
                        end else if (is_eol) begin
                            pstate <= P_IDLE;
                            if (cmd_w && nib == 4'd10 && w_ok) begin
                                case (w_addr)
                                    8'h01: begin run <= w_data[0]; pps_mode <= w_data[1]; end
                                    8'h02: win_len <= w_data[23:0];
                                    8'h18: fan_duty <= w_data[7:0];
                                    8'h40: sf_thresh <= w_data[11:0];
                                    8'h41: sf_holdoff <= w_data[15:0];
                                    default: ftw_bus[w_addr[2:0]*32 +: 32] <= w_data;   // 10..17
                                endcase
                                txbuf <= {"K", 8'h0A, 56'd0}; txcnt <= 4'd2;
                            end else if (!cmd_w && nib == 4'd2) begin
                                if (r_addr == 8'h44) sf_ack <= 1'b1;
                                if (r_addr == 8'h03) begin
                                    sh_seq   <= seq;
                                    sh_n     <= w_n;
                                    sh_ndec  <= w_ndec;
                                    sh_clip  <= w_clip;
                                    sh_flags <= w_flags;
                                    sh_min   <= w_min;
                                    sh_max   <= w_max;
                                    sh_sq    <= w_sq;
                                    sh_sum   <= w_sum;
                                    sh_pow   <= w_pow_bus;
                                    sh_pmax  <= w_pmax_bus;
                                end
                                txbuf <= {hex_chr(rdata[31:28]), hex_chr(rdata[27:24]),
                                          hex_chr(rdata[23:20]), hex_chr(rdata[19:16]),
                                          hex_chr(rdata[15:12]), hex_chr(rdata[11:8]),
                                          hex_chr(rdata[7:4]),   hex_chr(rdata[3:0]), 8'h0A};
                                txcnt <= 4'd9;
                            end else begin
                                txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 4'd2;
                            end
                        end else begin
                            pstate <= P_ERR;
                        end
                    end
                    default: begin
                        if (is_eol) begin
                            pstate <= P_IDLE;
                            txbuf <= {"E", 8'h0A, 56'd0}; txcnt <= 4'd2;
                        end
                    end
                endcase
            end

            // ---------------- transmitter
            case (tstate)
                T_IDLE: if (!f_empty && !tx_busy) begin
                    tx_data <= fifo[f_rp[8:0]];
                    tx_send <= 1'b1;
                    f_rp    <= f_rp + 10'd1;
                    tstate  <= T_WAIT1;
                end
                T_WAIT1: tstate <= T_WAIT2;               // uart_tx raises busy this clock
                T_WAIT2: if (!tx_busy) tstate <= T_IDLE;
                default: tstate <= T_IDLE;
            endcase
        end
    end

endmodule
