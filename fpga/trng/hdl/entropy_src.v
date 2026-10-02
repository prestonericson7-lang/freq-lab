`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// entropy_src.v -- ring-oscillator noise source (list item #73)
//
// Synthesis: NOSC free-running ring oscillators (odd inverter chains) run
// uncorrelated with the 50 MHz system clock. Each is sampled through two
// metastability-hardening flip-flops; the NOSC sampled bits are XORed, and
// DECIM of those XORed samples are XOR-accumulated into one raw entropy bit.
// The entropy is the jitter between the oscillators and the sampling clock.
//
// Simulation: ring oscillators are combinational loops and cannot be simulated,
// so a backdoor (sim_bit / sim_stb) feeds a testbench-chosen bit stream. This
// lets the whole downstream pipeline -- health tests, von Neumann debiaser,
// conditioner, packer, UART -- be checked bit-exactly against a Python model
// with known-balanced, known-biased and stuck inputs.
//
// raw_stb pulses once per output raw bit; raw_bit is valid that clock.
//-----------------------------------------------------------------------------
module entropy_src #(
    parameter integer NOSC  = 16,       // number of ring oscillators
    parameter integer RLEN  = 3,        // inverters per ring (odd)
    parameter integer DECIM = 8         // sampled bits XORed per raw bit
)(
    input  wire clk,
    input  wire rst_n,
    input  wire en,

    input  wire sim_bit,                // simulation backdoor
    input  wire sim_stb,

    output reg  raw_bit,
    output reg  raw_stb
);

`ifdef SIMULATION
    // ---- deterministic path: the testbench supplies the raw bits ----
    always @(posedge clk) begin
        if (!rst_n) begin raw_bit <= 1'b0; raw_stb <= 1'b0; end
        else begin
            raw_stb <= sim_stb & en;
            if (sim_stb & en) raw_bit <= sim_bit;
        end
    end

`else
    // ---- ring oscillators ----
    wire [NOSC-1:0] osc;
    genvar g;
    generate
        for (g = 0; g < NOSC; g = g + 1) begin : g_ro
            (* ALLOW_COMBINATORIAL_LOOPS = "TRUE", KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
            wire [RLEN:0] chain;
            // gate the loop with en so it can be stopped; odd chain -> oscillates
            assign chain[0] = en & ~chain[RLEN];
            for (genvar j = 0; j < RLEN; j = j + 1) begin : g_inv
                (* KEEP = "TRUE", DONT_TOUCH = "TRUE" *)
                assign chain[j+1] = ~chain[j];
            end
            assign osc[g] = chain[RLEN];
        end
    endgenerate

    // two-FF synchroniser per oscillator, then XOR
    reg [NOSC-1:0] s1, s2;
    always @(posedge clk) begin
        s1 <= osc;
        s2 <= s1;
    end
    wire xored = ^s2;

    // XOR-accumulate DECIM samples into one raw bit
    reg [$clog2(DECIM):0] dcnt;
    reg acc;
    always @(posedge clk) begin
        if (!rst_n) begin dcnt <= 0; acc <= 1'b0; raw_bit <= 1'b0; raw_stb <= 1'b0; end
        else begin
            raw_stb <= 1'b0;
            if (en) begin
                acc <= acc ^ xored;
                if (dcnt == DECIM - 1) begin
                    dcnt <= 0;
                    raw_bit <= acc ^ xored;
                    raw_stb <= 1'b1;
                    acc <= 1'b0;
                end else begin
                    dcnt <= dcnt + 1'b1;
                end
            end
        end
    end
`endif

endmodule
