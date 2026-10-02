`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// uart.v -- plain 8N1 UART receiver and transmitter
//
// CLKS_PER_BIT = clock frequency / baud rate. 50 MHz / 115200 = 434.
//-----------------------------------------------------------------------------
module uart_rx #(
    parameter integer CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,
    output reg  [7:0] data,
    output reg        valid      // one clock pulse per received byte
);
    localparam integer CW = $clog2(CLKS_PER_BIT + 1);

    reg [1:0]    sync;
    reg [CW-1:0] cnt;
    reg [3:0]    bit_idx;
    reg [7:0]    shreg;
    reg          active;

    always @(posedge clk) begin
        if (!rst_n) begin
            sync    <= 2'b11;
            cnt     <= {CW{1'b0}};
            bit_idx <= 4'd0;
            shreg   <= 8'd0;
            active  <= 1'b0;
            valid   <= 1'b0;
            data    <= 8'd0;
        end else begin
            valid <= 1'b0;
            sync  <= {sync[0], rx};
            if (!active) begin
                if (!sync[1]) begin                       // start bit seen
                    active  <= 1'b1;
                    cnt     <= CLKS_PER_BIT / 2;          // sample mid-bit
                    bit_idx <= 4'd0;
                end
            end else if (cnt != {CW{1'b0}}) begin
                cnt <= cnt - 1'b1;
            end else begin
                cnt <= CLKS_PER_BIT - 1;
                if (bit_idx == 4'd0) begin
                    if (sync[1]) active <= 1'b0;          // false start
                    else         bit_idx <= 4'd1;
                end else if (bit_idx <= 4'd8) begin
                    shreg   <= {sync[1], shreg[7:1]};     // LSB first
                    bit_idx <= bit_idx + 4'd1;
                end else begin
                    active <= 1'b0;                       // stop bit
                    if (sync[1]) begin
                        data  <= shreg;
                        valid <= 1'b1;
                    end
                end
            end
        end
    end
endmodule


module uart_tx #(
    parameter integer CLKS_PER_BIT = 434
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [7:0] data,
    input  wire       send,      // pulse while !busy
    output reg        tx,
    output wire       busy
);
    localparam integer CW = $clog2(CLKS_PER_BIT + 1);

    reg [CW-1:0] cnt;
    reg [3:0]    bits_left;
    reg [9:0]    shreg;

    assign busy = (bits_left != 4'd0);

    always @(posedge clk) begin
        if (!rst_n) begin
            tx        <= 1'b1;
            cnt       <= {CW{1'b0}};
            bits_left <= 4'd0;
            shreg     <= 10'h3FF;
        end else if (bits_left == 4'd0) begin
            tx <= 1'b1;
            if (send) begin
                shreg     <= {1'b1, data, 1'b0};          // stop, data (LSB first), start
                bits_left <= 4'd10;
                cnt       <= CLKS_PER_BIT - 1;
                tx        <= 1'b0;                        // start bit goes out now
            end
        end else if (cnt != {CW{1'b0}}) begin
            cnt <= cnt - 1'b1;
        end else begin
            cnt       <= CLKS_PER_BIT - 1;
            bits_left <= bits_left - 4'd1;
            shreg     <= {1'b1, shreg[9:1]};
            tx        <= (bits_left == 4'd1) ? 1'b1 : shreg[1];
        end
    end
endmodule
