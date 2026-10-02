`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// xadc_dual.v -- Zynq XADC, two auxiliary channels, sequencer mode
//
// Channels: VAUX1 (E17/D18 = JM1 pins 6/8) and VAUX9 (E18/E19 = JM1 pins 9/11).
// The sequencer alternates between VAUX1 and VAUX9, so each channel is
// sampled at half the aggregate rate:  ~195 kS/s per channel at DCLK/4.
//
// Both channels are synchronous to the sequencer, so the A and B strobes
// are one conversion apart (~2.56 us at 390 kS/s aggregate).  For the
// correlator this is negligible up to about 50 kHz signal bandwidth.
//
// REGISTER SETUP (UG480, 7 Series XADC)
//   40h  0x0100  continuous sequencer, unipolar, ACQ=1
//   41h  0x20EF  sequencer mode, alarms off
//   42h  0x0400  ADC clock divider 4
//   48h  0x0000  no on-chip channels in the sequence
//   49h  0x0202  sequence VAUX1 + VAUX9  (49h bit n = VAUXn; in 48h, bit 9
//                would select VCCINT instead -- UG480 sequencer registers)
//
// For simulation (define SIMULATION) the primitive is replaced by a model.
//-----------------------------------------------------------------------------
module xadc_dual (
    input  wire        clk,
    input  wire        rst,
    input  wire        vauxp1,
    input  wire        vauxn1,
    input  wire        vauxp9,
    input  wire        vauxn9,
    output reg  [11:0] data_a,
    output reg         stb_a,
    output reg  [11:0] data_b,
    output reg         stb_b
);

`ifdef SIMULATION

    reg [11:0] sim_code_a = 12'd2048;
    reg [11:0] sim_code_b = 12'd2048;
    reg [7:0]  cnt        = 8'd0;
    reg        which      = 1'b0;         // 0 = A, 1 = B

    always @(posedge clk) begin
        stb_a <= 1'b0;
        stb_b <= 1'b0;
        if (rst) begin
            cnt   <= 8'd0;
            which <= 1'b0;
        end else if (cnt == 8'd127) begin
            cnt   <= 8'd0;
            which <= ~which;
            if (!which) begin
                data_a <= sim_code_a;
                stb_a  <= 1'b1;
            end else begin
                data_b <= sim_code_b;
                stb_b  <= 1'b1;
            end
        end else begin
            cnt <= cnt + 8'd1;
        end
    end

`else

    wire [15:0] do_w;
    wire        drdy_w;
    wire        eoc_w;
    wire [4:0]  chan_w;

    XADC #(
        .INIT_40 (16'h0100),   // continuous seq, unipolar, ACQ=1
        .INIT_41 (16'h20EF),   // sequencer on, alarms off
        .INIT_42 (16'h0400),   // DCLK/4
        .INIT_48 (16'h0000),   // no on-chip channels
        .INIT_49 (16'h0202),   // VAUX1 (bit 1) + VAUX9 (bit 9)
        .INIT_4A (16'h0000), .INIT_4B (16'h0000),
        .INIT_4C (16'h0000), .INIT_4D (16'h0000),
        .INIT_4E (16'h0000), .INIT_4F (16'h0000),
        .INIT_50 (16'h0000), .INIT_51 (16'h0000),
        .INIT_52 (16'h0000), .INIT_53 (16'h0000),
        .INIT_54 (16'h0000), .INIT_55 (16'h0000),
        .INIT_56 (16'h0000), .INIT_57 (16'h0000),
        .INIT_58 (16'h0000), .INIT_5C (16'h0000),
        .SIM_DEVICE ("7SERIES")
    ) u_xadc (
        .DCLK         (clk),
        .RESET        (rst),
        .DEN          (eoc_w),
        .DWE          (1'b0),
        .DADDR        ({2'b00, chan_w}),
        .DI           (16'h0000),
        .DO           (do_w),
        .DRDY         (drdy_w),
        .CONVST       (1'b0),
        .CONVSTCLK    (1'b0),
        .VP           (1'b0),
        .VN           (1'b0),
        .VAUXP        ({6'b0, vauxp9, 7'b0, vauxp1, 1'b0}),  // bits 9 and 1
        .VAUXN        ({6'b0, vauxn9, 7'b0, vauxn1, 1'b0}),
        .ALM          (),
        .OT           (),
        .BUSY         (),
        .CHANNEL      (chan_w),
        .EOC          (eoc_w),
        .EOS          (),
        .JTAGBUSY     (),
        .JTAGLOCKED   (),
        .JTAGMODIFIED (),
        .MUXADDR      ()
    );

    // route DRP result to the right output
    always @(posedge clk) begin
        stb_a <= 1'b0;
        stb_b <= 1'b0;
        if (drdy_w) begin
            if (chan_w == 5'd17) begin          // 0x11 = VAUX1
                data_a <= do_w[15:4];
                stb_a  <= 1'b1;
            end else if (chan_w == 5'd25) begin  // 0x19 = VAUX9
                data_b <= do_w[15:4];
                stb_b  <= 1'b1;
            end
        end
    end

`endif

endmodule
