`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// coincidence.v -- coincidence counter for two photon detector channels
//
// Counts singles on each channel, and coincidences: an edge on A and an edge
// on B in the same clock, or within WINDOW clock ticks of each other in
// either order -- |t_A - t_B| <= WINDOW ticks, i.e. 2*WINDOW+1 clock bins.
//
// After the run, the host computes:
//   accidentals       = singles_a * singles_b * (2*window + 1) / elapsed_clk
//   true_coincidences = coincidences - accidentals
//
// The counters are 48 bits wide.  At 1 million counts/second, overflow
// takes ~9 years.
//
// The coincidence detection uses a simple approach: when an edge arrives on
// one channel, a countdown timer starts (set to WINDOW).  If the other
// channel fires in the same clock, or before the countdown reaches zero,
// it's a coincidence.  Each pulse takes part in at most one coincidence:
// the two pulses of a pair do not open new windows, and both windows close.
//-----------------------------------------------------------------------------
module coincidence (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        edge_a,
    input  wire        edge_b,

    input  wire        start,
    input  wire        stop,
    input  wire [15:0] window,        // coincidence window in clock cycles

    output reg         busy,
    output reg         done,
    output reg  [47:0] singles_a,
    output reg  [47:0] singles_b,
    output reg  [47:0] coincidences,
    output reg  [31:0] elapsed_clk
);

    reg [15:0] timer_ab = 0;         // countdown: A fired, waiting for B
    reg [15:0] timer_ba = 0;         // countdown: B fired, waiting for A

    always @(posedge clk) begin
        if (!rst_n) begin
            busy        <= 1'b0;
            done        <= 1'b0;
            singles_a   <= 48'd0;
            singles_b   <= 48'd0;
            coincidences <= 48'd0;
            elapsed_clk <= 32'd0;
            timer_ab    <= 16'd0;
            timer_ba    <= 16'd0;
        end else begin
            done <= 1'b0;

            if (start && !busy) begin
                busy         <= 1'b1;
                singles_a    <= 48'd0;
                singles_b    <= 48'd0;
                coincidences <= 48'd0;
                elapsed_clk  <= 32'd0;
                timer_ab     <= 16'd0;
                timer_ba     <= 16'd0;
            end

            if (stop && busy) begin
                busy <= 1'b0;
                done <= 1'b1;
            end

            if (busy) begin
                elapsed_clk <= elapsed_clk + 32'd1;

                if (edge_a) singles_a <= singles_a + 48'd1;
                if (edge_b) singles_b <= singles_b + 48'd1;

                // A coincidence: both edges in the same clock, or an edge on
                // one channel while the other channel's window is still open.
                if ((edge_a && edge_b) ||
                    (edge_a && timer_ba != 16'd0) ||
                    (edge_b && timer_ab != 16'd0)) begin
                    coincidences <= coincidences + 48'd1;
                    timer_ab     <= 16'd0;        // the pair is used up:
                    timer_ba     <= 16'd0;        // close both windows
                end else begin
                    // a lone edge opens (or restarts) its window,
                    // otherwise an open window counts down
                    if (edge_a)                 timer_ab <= window;
                    else if (timer_ab != 16'd0) timer_ab <= timer_ab - 16'd1;
                    if (edge_b)                 timer_ba <= window;
                    else if (timer_ba != 16'd0) timer_ba <= timer_ba - 16'd1;
                end
            end
        end
    end

endmodule
