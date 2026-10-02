`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// xadc_vaux1.v -- the Zynq's built-in 12-bit ADC, one channel, free-running
//
// Channel : VAUX1  = balls E17 (P) / D18 (N) = JM1 pin 6 / pin 8
// Range   : 0 .. 1.0 V on the P pin, measured against the N pin (tie N to the
//           signal's ground at the source). MORE THAN 1.0 V READS AS FULL SCALE.
//           Never exceed the 3.3 V bank supply on either pin.
// Rate    : DCLK 50 MHz / 4 = 12.5 MHz ADC clock, 32 ADC clocks per conversion
//           (the extra-acquisition bit is set so sources up to about 20 kOhm
//           settle) = 390.625 kS/s.
//
// Register values are from Xilinx UG480 (7 Series XADC user guide):
//   40h = 0x0111  channel 0x11 (VAUX1), unipolar, continuous, ACQ = 1, no averaging
//   41h = 0x3F0F  single-channel mode, all alarms off, no calibration coefficients
//   42h = 0x0400  ADC clock divider 4, both ADCs powered
//
// Each end-of-conversion strobes a DRP read of the channel's result register;
// the 12-bit code is the top 12 bits of the 16-bit word.
//
// For simulation (define SIMULATION) the primitive is replaced by a model that
// samples `sim_code` at the same rate; a testbench drives it by hierarchical
// name.
//-----------------------------------------------------------------------------
module xadc_vaux1 (
    input  wire        clk,
    input  wire        rst,
    input  wire        vauxp1,
    input  wire        vauxn1,
    output reg  [11:0] data,
    output reg         stb
);

`ifdef SIMULATION

    reg [11:0] sim_code = 12'd2048;
    reg [6:0]  cnt      = 7'd0;

    always @(posedge clk) begin
        stb <= 1'b0;
        if (rst) begin
            cnt <= 7'd0;
        end else if (cnt == 7'd127) begin      // 128 clocks = 32 ADC clocks at /4
            cnt  <= 7'd0;
            data <= sim_code;
            stb  <= 1'b1;
        end else begin
            cnt <= cnt + 7'd1;
        end
    end

`else

    wire [15:0] do_w;
    wire        drdy_w;
    wire        eoc_w;
    wire [4:0]  chan_w;

    XADC #(
        .INIT_40 (16'h0111),
        .INIT_41 (16'h3F0F),
        .INIT_42 (16'h0400),
        .INIT_48 (16'h0000), .INIT_49 (16'h0000),
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
        .DEN          (eoc_w),                   // read the result as soon as it exists
        .DWE          (1'b0),
        .DADDR        ({2'b00, chan_w}),         // status register of the channel just converted
        .DI           (16'h0000),
        .DO           (do_w),
        .DRDY         (drdy_w),
        .CONVST       (1'b0),
        .CONVSTCLK    (1'b0),
        .VP           (1'b0),
        .VN           (1'b0),
        .VAUXP        ({14'b0, vauxp1, 1'b0}),   // bit 1 = VAUX1
        .VAUXN        ({14'b0, vauxn1, 1'b0}),
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

    always @(posedge clk) begin
        stb <= drdy_w;
        if (drdy_w) data <= do_w[15:4];
    end

`endif

endmodule
