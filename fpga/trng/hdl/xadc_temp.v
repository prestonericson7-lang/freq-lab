`timescale 1ns / 1ps
//-----------------------------------------------------------------------------
// xadc_temp.v -- read the Zynq XADC on-chip temperature and VCCINT sensors
// (list item #73: SRI logged the diode temperature to 0.2 C). No external pins.
//
//   temp_code  : 12-bit on-chip temperature (host: T_C = code*503.975/4096 - 273.15)
//   vccint_code: 12-bit VCCINT              (host: V = code*3.0/4096)
//
// For simulation (SIMULATION) the primitive is replaced by fixed codes so the
// testbench and the register file can be exercised without the XADC model.
//-----------------------------------------------------------------------------
module xadc_temp (
    input  wire        clk,
    input  wire        rst,
    output reg  [11:0] temp_code,
    output reg  [11:0] vccint_code
);
`ifdef SIMULATION
    initial begin temp_code = 12'h97C; vccint_code = 12'h555; end   // ~27 C, ~1.0 V
    always @(posedge clk) begin end
`else
    wire [15:0] do_w;
    wire        drdy_w, eoc_w;
    wire [4:0]  chan_w;
    reg  [6:0]  addr;

    // cycle the DRP address between temp (0x00) and vccint (0x01) on each DRDY
    always @(posedge clk) begin
        if (rst) begin addr <= 7'h00; temp_code <= 12'd0; vccint_code <= 12'd0; end
        else if (drdy_w) begin
            if (addr == 7'h00) temp_code   <= do_w[15:4];
            else               vccint_code <= do_w[15:4];
            addr <= (addr == 7'h00) ? 7'h01 : 7'h00;
        end
    end

    XADC #(
        .INIT_40(16'h0000), .INIT_41(16'h2F0F), .INIT_42(16'h0400),
        .INIT_48(16'h0700), .INIT_49(16'h0000),
        .INIT_4A(16'h0000), .INIT_4B(16'h0000),
        .SIM_DEVICE("7SERIES")
    ) u_xadc (
        .DCLK(clk), .RESET(rst),
        .DEN(eoc_w), .DWE(1'b0), .DADDR({2'b00, addr[4:0]}), .DI(16'h0000),
        .DO(do_w), .DRDY(drdy_w),
        .CONVST(1'b0), .CONVSTCLK(1'b0),
        .VP(1'b0), .VN(1'b0), .VAUXP(16'd0), .VAUXN(16'd0),
        .ALM(), .OT(), .BUSY(), .CHANNEL(chan_w), .EOC(eoc_w), .EOS(),
        .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED(), .MUXADDR()
    );
`endif
endmodule
