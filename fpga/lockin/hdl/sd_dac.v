`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// sd_dac.v -- first-order sigma-delta DAC, one output pin
//
// The pin toggles at up to 50 MHz; its average is the 16-bit input. An RC
// low-pass on the pin (1 kOhm + 10 nF, corner about 16 kHz) turns it into an
// analog voltage: 0x0000 = 0 V, 0xFFFF = 3.3 V on an LVCMOS33 pin.
// First order is unconditionally stable for any input.
//-----------------------------------------------------------------------------
module sd_dac (
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] din,     // unsigned, offset binary (0x8000 = mid-scale)
    output reg         dout
);
    reg [16:0] acc;

    always @(posedge clk) begin
        if (!rst_n) begin
            acc  <= 17'd0;
            dout <= 1'b0;
        end else begin
            acc  <= {1'b0, acc[15:0]} + {1'b0, din};
            dout <= acc[16];
        end
    end
endmodule
