`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// cordic_sincos.v -- pipelined CORDIC, phase in -> sin and cos out
//
// No ROM file and no vendor IP: pure logic, so it builds the same in Vivado,
// openXC7 and a simulator. Sine and cosine come out of the same pipeline, which
// is what a lock-in needs (the drive and both references are one signal).
//
//   phase : 20 bits, 2^20 = one full cycle
//   sin_o, cos_o : signed 16 bit, amplitude about +-32700
//   latency : LATENCY clocks (input register + 16 iterations + output register)
//-----------------------------------------------------------------------------
module cordic_sincos (
    input  wire               clk,
    input  wire [19:0]        phase,
    output reg  signed [15:0] sin_o,
    output reg  signed [15:0] cos_o
);
    localparam integer PW      = 20;
    localparam integer N       = 16;
    localparam integer XW      = 19;
    localparam integer LATENCY = N + 2;

    // 4 * 32700 / K, K = 1.6467602579 (CORDIC gain for 16 iterations).
    // Two extra fraction bits are carried through the iterations.
    localparam signed [XW-1:0] A0 = 19'sd79429;

    // atan(2^-i) in phase units (2^20 per cycle), i = 0 in the low 20 bits.
    localparam [PW*N-1:0] ATAN = {
        20'd5,     20'd10,    20'd20,    20'd41,
        20'd81,    20'd163,   20'd326,   20'd652,
        20'd1304,  20'd2607,  20'd5213,  20'd10417,
        20'd20753, 20'd40884, 20'd77376, 20'd131072 };

    reg signed [XW-1:0] x [0:N];
    reg signed [XW-1:0] y [0:N];
    reg signed [PW-1:0] z [0:N];

    // Quadrant folding: CORDIC converges for angles within +-90 degrees, so
    // angles in the second and third quadrants are turned by 180 degrees and
    // the start vector is negated.
    wire flip = phase[PW-1] ^ phase[PW-2];

    always @(posedge clk) begin
        x[0] <= flip ? -A0 : A0;
        y[0] <= {XW{1'b0}};
        z[0] <= flip ? {~phase[PW-1], phase[PW-2:0]} : phase;
    end

    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : g_iter
            wire signed [PW-1:0] a = ATAN[PW*i +: PW];
            always @(posedge clk) begin
                if (!z[i][PW-1]) begin          // residual angle >= 0: rotate positive
                    x[i+1] <= x[i] - (y[i] >>> i);
                    y[i+1] <= y[i] + (x[i] >>> i);
                    z[i+1] <= z[i] - a;
                end else begin                   // residual angle < 0: rotate negative
                    x[i+1] <= x[i] + (y[i] >>> i);
                    y[i+1] <= y[i] - (x[i] >>> i);
                    z[i+1] <= z[i] + a;
                end
            end
        end
    endgenerate

    // drop the two fraction bits with rounding
    wire signed [XW-1:0] xr = x[N] + 19'sd2;
    wire signed [XW-1:0] yr = y[N] + 19'sd2;

    always @(posedge clk) begin
        cos_o <= xr[17:2];
        sin_o <= yr[17:2];
    end

endmodule
