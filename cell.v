//=========================================================================
// Sorting Accelerator Cell Implementation
//=========================================================================

`ifndef LAB2_SORT_XCEL_CELL_V
`define LAB2_SORT_XCEL_CELL_V

`include "vc/muxes.v"
`include "vc/regs.v"
`include "vc/arithmetic.v"

module sort_xcel_cell
(
  input  logic clk,
  input  logic reset,

  input  logic [31:0] in0,
  input  logic [31:0] in1,
  output logic [31:0] out,

  input  logic in_sel,
  input  logic reg_en,
  output logic comp_result
);

logic [31:0] reg_in;

vc_Mux2#(32) in_mux
(
  .sel (in_sel),
  .in0 (in0),
  .in1 (in1),
  .out (reg_in)
);

vc_EnResetReg#(32,0) cell_reg
(
  .clk   (clk),
  .reset (reset),
  .d     (reg_in),
  .q     (out),
  .en    (reg_en)
);

vc_GtComparator#(32) comp
(
  .in0 (out),
  .in1 (in1),
  .out (comp_result)
);

endmodule

`endif /* LAB2_SORT_XCEL_CELL_V */