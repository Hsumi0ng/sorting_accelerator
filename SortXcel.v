//=========================================================================
// Sorting Accelerator Implementation
//=========================================================================
// Sort array in memory containing positive integers.
// Accelerator register interface:
//
//  xr0 : go/done
//  xr1 : base address of array
//  xr2 : number of elements in array
//
// Accelerator protocol involves the following steps:
//  1. Write the base address of array via xr1
//  2. Write the number of elements in array via xr2
//  3. Tell accelerator to go by writing xr0
//  4. Wait for accelerator to finish by reading xr0, result will be 1
//

`ifndef LAB2_SORT_SORT_XCEL_V
`define LAB2_SORT_SORT_XCEL_V

`include "vc/trace.v"

`include "vc/mem-msgs.v"
`include "vc/xcel-msgs.v"
`include "vc/queues.v"

`include "lab2_xcel/cell.v"
`include "proc/XcelMsg.v"

//=========================================================================
// Sorting Accelerator Implementation
//=========================================================================

module lab2_xcel_SortXcel
(
  input  logic         clk,
  input  logic         reset,

  input  xcel_req_t    xcel_reqstream_msg,
  input  logic         xcel_reqstream_val,
  output logic         xcel_reqstream_rdy,

  output xcel_resp_t   xcel_respstream_msg,
  output logic         xcel_respstream_val,
  input  logic         xcel_respstream_rdy,

  output mem_req_4B_t  mem_reqstream_msg,
  output logic         mem_reqstream_val,
  input  logic         mem_reqstream_rdy,

  input  mem_resp_4B_t mem_respstream_msg,
  input  logic         mem_respstream_val,
  output logic         mem_respstream_rdy
);

  // 4-state sim fix: force outputs to be zero if invalid

  xcel_resp_t  xcel_respstream_msg_raw;
  mem_req_4B_t mem_reqstream_msg_raw;

  assign xcel_respstream_msg = xcel_respstream_msg_raw & {33{xcel_respstream_val}};
  assign mem_reqstream_msg   = mem_reqstream_msg_raw & {78{mem_reqstream_val}};

  // Accelerator ports and queues

  logic      xcelreq_deq_val;
  logic      xcelreq_deq_rdy;
  xcel_req_t xcelreq_deq_msg;

  vc_Queue#(`VC_QUEUE_PIPE,$bits(xcel_req_t),1) xcelreq_q
  (
    .clk     (clk),
    .reset   (reset),
    .num_free_entries(),

    .enq_val (xcel_reqstream_val),
    .enq_rdy (xcel_reqstream_rdy),
    .enq_msg (xcel_reqstream_msg),

    .deq_val (xcelreq_deq_val),
    .deq_rdy (xcelreq_deq_rdy),
    .deq_msg (xcelreq_deq_msg)
  );

  // Memory ports and queues

  logic         memresp_deq_val;
  logic         memresp_deq_rdy;
  mem_resp_4B_t memresp_deq_msg;

  vc_Queue#(`VC_QUEUE_PIPE,$bits(mem_resp_4B_t),1) memresp_q
  (
    .clk     (clk),
    .reset   (reset),
    .num_free_entries(),

    .enq_val (mem_respstream_val),
    .enq_rdy (mem_respstream_rdy),
    .enq_msg (mem_respstream_msg),

    .deq_val (memresp_deq_val),
    .deq_rdy (memresp_deq_rdy),
    .deq_msg (memresp_deq_msg)
  );

  //----------------------------------------------------------------------
  // Data Path
  //----------------------------------------------------------------------

  localparam num_cell = 128;

  // States declaration

  typedef enum logic [$clog2(5)-1:0] {
    STATE_XCFG,
    STATE_CALC,
    STATE_WRITE,
    STATE_READ,
    STATE_MERGE
  } state_t;

  state_t state_reg, state_next;

  // Input registers, xr0, xr1, and xr2

  logic        done;
  logic        xr0_reset;
  logic [31:0] base_addr_in;
  logic [31:0] size_in;
  logic [31:0] base_addr;
  logic [31:0] size;
  logic [9:0]  count;

  assign xr0_reset = reset || (xcel_respstream_val && xcel_respstream_rdy
  && xcel_respstream_msg_raw.type_  == `XcelRespMsg_TYPE_READ);

  vc_EnResetReg#(1,0) xr0
  (
    .clk   (clk),
    .reset (xr0_reset),
    .d     (1'b1),
    .q     (done),
    .en    (state_reg == STATE_WRITE && count == 10'b1 && mem_reqstream_rdy)
  );

  vc_EnResetReg#(32,0) xr1
  (
    .clk   (clk),
    .reset (xr0_reset),
    .d     (base_addr_in),
    .q     (base_addr),
    .en    (state_reg == STATE_XCFG)
  );

  vc_EnResetReg#(32,0) xr2
  (
    .clk   (clk),
    .reset (xr0_reset),
    .d     (size_in),
    .q     (size),
    .en    (state_reg == STATE_XCFG)
  );

  // Cells

  logic [31:0] in;
  logic [31:0] out[num_cell];
  logic        sel[num_cell];
  logic        reg_en[num_cell + 1];
  logic        comp_result[num_cell];

  sort_xcel_cell cell_unit0
  (
    .clk         (clk) ,
    .reset       (xr0_reset),
    .in0         (in),
    .in1         (in),
    .out         (out[0]),
    .in_sel      (1'b0),
    .reg_en      (reg_en[0]),
    .comp_result (comp_result[0])
  );

  genvar i;
  generate
  for (i=1; i<num_cell; i=i+1) begin
    sort_xcel_cell cell_unit
    (
      .clk         (clk) ,
      .reset       (xr0_reset),
      .in0         (out[i-1]),
      .in1         (in),
      .out         (out[i]),
      .in_sel      (sel[i]),
      .reg_en      (reg_en[i]),
      .comp_result (comp_result[i])
    );
  end
  endgenerate

  // Register used in STATE_MERGE

  logic [31:0] merge_data;
  logic [31:0] merge_in;

  always@(*) begin
    if (!comp_result[count-1]) merge_in = in;
    else merge_in = out[count-1];
  end

  vc_EnResetReg#(32,0) merge_reg
  (
    .clk   (clk),
    .reset (xr0_reset),
    .d     (merge_in),
    .q     (merge_data),
    .en    (reg_en[count])
  );

  //----------------------------------------------------------------------
  // Control Unit
  //----------------------------------------------------------------------

  logic [2:0]  state_last;
  logic [2:0]  state_last1;
  logic [31:0] idx;
  logic [31:0] addr_last;
  logic [31:0] mem_respstream_last;
  logic        memresp_effect;
  logic        memreq_sent;
  logic        addr_change;
  logic        memreq_sent_first;
  logic        sent_first_reset;  // Reset when the xcel is about to jump from STATE_CALC to STATE_WRITE or STATE_READ
  logic [31:0] num_written;
  logic [31:0] read_count;
  logic [31:0] temp;
  logic        have_written;

  //======================================================================
  // State Update
  //======================================================================

  logic go;

  always_ff @(posedge clk) begin
    if ( reset )
      state_reg <= STATE_XCFG;
    else 
      state_reg <= state_next;
  end

  always@(*) begin

      case ( state_reg )

        STATE_XCFG:
          if (go && !done)
            state_next = STATE_CALC;
          else 
            state_next = STATE_XCFG;

        STATE_CALC:
          if (memresp_deq_val) begin
            if ((count == size-1 || count == num_cell-1) && !have_written)
              state_next = STATE_WRITE;
            else if (count == num_cell-1 || (idx == size && count == idx - num_written - 1))
              state_next = STATE_READ;
            else
              state_next = STATE_CALC;
          end
          else
            state_next = STATE_CALC;
        
        STATE_WRITE:
          if ( count == 10'b1 && mem_reqstream_rdy && idx == size)
            state_next = STATE_XCFG;
          else if (count == 10'b1 && mem_reqstream_rdy && idx != size)
            state_next = STATE_CALC;
          else 
            state_next = STATE_WRITE;
        
        STATE_READ:
          if (read_count == 0)
            state_next = STATE_READ;
          else if (addr_change)
            state_next = STATE_MERGE;
          else 
            state_next = STATE_READ;
        
        STATE_MERGE:
          if (mem_reqstream_msg.addr != addr_last) begin
            if (mem_reqstream_rdy && read_count <= num_written && mem_respstream_val && mem_respstream_msg.type_ == `VC_MEM_RESP_MSG_TYPE_READ)
              state_next = STATE_READ;
            else if (mem_reqstream_rdy && read_count == num_written+1)
              state_next = STATE_WRITE;
            else 
              state_next = STATE_MERGE;
          end
          else begin
            if (mem_reqstream_rdy && read_count <= num_written && mem_respstream_val && mem_respstream_msg.type_ == `VC_MEM_RESP_MSG_TYPE_READ)
              state_next = STATE_READ;
            else if (memreq_sent && read_count <= num_written && mem_respstream_val && mem_respstream_msg.type_ == `VC_MEM_RESP_MSG_TYPE_READ)
              state_next = STATE_READ;
            else if (memresp_effect && mem_reqstream_rdy && read_count <= num_written)
              state_next = STATE_READ;
            else if (mem_reqstream_rdy && read_count == num_written+1)
              state_next = STATE_WRITE;
            else
              state_next = STATE_MERGE;
          end
            
        default:
          state_next = STATE_XCFG;

      endcase
    end
  
  // Save last state

  always@(posedge clk) begin
    if (xr0_reset) state_last <= STATE_XCFG;
    else if (state_reg != state_next) state_last <= state_reg; // Save current state when about to jump to another state
    else state_last <= state_last;
  end

  always@(posedge clk) begin
    if (xr0_reset) state_last1 <= STATE_XCFG;
    else state_last1 <= state_reg;
  end

  // Index

  always@(posedge clk) begin
    if (xr0_reset) mem_respstream_last <= 32'b0;
    else mem_respstream_last <= mem_respstream_msg.data;
  end

  always@(posedge clk) begin
    if (xr0_reset) addr_last <= 32'b0;
    else addr_last <= mem_reqstream_msg_raw.addr;
  end

  always@(posedge clk) begin
    if (xr0_reset) memresp_effect <= 0;
    else if (mem_reqstream_msg.addr != addr_last && mem_respstream_last == mem_respstream_msg.data) memresp_effect <= 0;
    else if (mem_respstream_last != mem_respstream_msg.data && mem_respstream_val) memresp_effect <= 1;
    else memresp_effect <= memresp_effect;
  end

  always@(posedge clk) begin
    if (xr0_reset || state_reg == STATE_XCFG) memreq_sent <= 0;
    else if (mem_reqstream_msg_raw.addr != addr_last && !(mem_reqstream_rdy && mem_reqstream_val)) memreq_sent <= 0;
    else if (mem_reqstream_val && mem_reqstream_rdy) memreq_sent <= 1;
    else memreq_sent <= memreq_sent;
  end

  always@(posedge clk) begin
    if (xr0_reset || state_reg == STATE_XCFG) idx <= 32'b0;
    else if (state_reg == STATE_READ) idx <= idx;
    else if (addr_change) idx <= idx + 1;
    else idx <= idx;
  end

  always@(*) begin
    if ((idx < size && state_reg == STATE_CALC) || (state_reg == STATE_READ)) begin
      if (read_count == 32'b1 && mem_reqstream_rdy)
        addr_change = 1;
      else if (state_reg == STATE_READ && read_count == num_written)
        addr_change = mem_respstream_val;
      else if (idx == 32'b0) begin
        if (mem_reqstream_rdy) addr_change = 1;
        else addr_change = 0;
      end
      else begin
        if (mem_reqstream_msg_raw.addr != addr_last)
          addr_change = mem_respstream_val && mem_reqstream_rdy && mem_reqstream_val;
        else if (mem_respstream_val && mem_reqstream_rdy && mem_reqstream_val) addr_change = 1;
        else if (memreq_sent) addr_change = mem_respstream_val;
        else if (memresp_effect) addr_change = mem_reqstream_rdy && mem_reqstream_val;
        else addr_change = 0;
      end
    end
    else addr_change = 0;
  end

  // Counter

  always@(posedge clk) begin
    if (xr0_reset) count <= 10'b0;
    else if (state_reg == STATE_CALC && memresp_deq_val && memresp_deq_msg.type_ == `VC_MEM_RESP_MSG_TYPE_READ)
      count <= count + 1;
    else if (state_reg == STATE_WRITE && mem_reqstream_rdy) count <= count - 1;
    else count <= count;
  end

  // Register write enable
  
  logic [10:0] j;
  always@(*) begin
    for (j=0; j<=count; j=j+1) begin 
      if ((state_reg == STATE_CALC || state_reg == STATE_READ) && 
      (memresp_deq_val || (state_last1 == STATE_MERGE))) begin
        if (memresp_deq_msg.type_ == `VC_MEM_RESP_MSG_TYPE_WRITE) reg_en[j] = 0;
        else if ((count == 10'b0) && (j == 0)) reg_en[j] = 1; // Load the first word
        else if (!comp_result[count-1]) begin  // The input is greater than all sorted values
          if (j == count) reg_en[j] = 1;
          else reg_en[j] = 0;
        end
        else begin
          if (j == count) reg_en[j] = 1;
          else if (!comp_result[j]) reg_en[j] = 0;
          else reg_en[j] = 1;
        end
      end
      else reg_en[j] = 0;
    end
  end

  // Register write selection
  
  logic [10:0] k;
  always@(*) begin
    for (k=1; k<=count; k=k+1) begin
      if (!comp_result[count-1]) begin
        if (k == count) sel[k] = 1;
        else sel[k] = 0;
      end
      else if (comp_result[0]) sel[k] = 0;
      else begin
        if (k == count) sel[k] = 0;
        else if (comp_result[k-1] ^ comp_result[k]) sel[k] = 1;
        else sel[k] = 0;
      end
    end
  end

  // Register to store whether the first memory request is sent

  assign sent_first_reset = (memresp_deq_val) && (count == num_cell-1 || (idx == size && count == idx - num_written - 1));

  always@(posedge clk) begin
    if (xr0_reset || sent_first_reset || state_reg == STATE_WRITE) memreq_sent_first <= 0;
    else if (mem_reqstream_val && mem_reqstream_rdy) memreq_sent_first <= 1;
    else memreq_sent_first <= memreq_sent_first;
  end

  // Register to store how many values have been written into the cache

  always@(posedge clk) begin
    if (xr0_reset) num_written <= 32'b0;
    else if (state_reg == STATE_WRITE && mem_reqstream_rdy) num_written <= num_written + 1;
    else num_written <= num_written;
  end

  // Register to store how many values have been read in the STATE_READ

  always@(posedge clk) begin
    if (xr0_reset || state_reg == STATE_CALC) 
      read_count <= 32'b0;
    else if (state_reg == STATE_READ && read_count == 0 && mem_respstream_val)
      read_count <= read_count + 1;
    else if (addr_change)
      read_count <= read_count + 1;
    else
      read_count <= read_count;
  end

  // Register to store a temp, using it in STATE_WRITE

  always@(posedge clk) begin
    if (xr0_reset || (state_reg == STATE_WRITE && state_next == STATE_CALC)) temp <= 32'b0;
    else if (state_reg == STATE_CALC && (state_next == STATE_WRITE || state_next == STATE_READ)) 
      temp <= count + 1;
    else temp <= temp;
  end

  // Register to store whether the xcel has been to STATE_WRITE

  always@(posedge clk) begin
    if (xr0_reset) have_written <= 0;
    else if (state_reg == STATE_WRITE) have_written <= 1;
    else have_written <= have_written;
  end

  // Temporary

  always@(*) begin
  
  xcel_respstream_val = 0;
  mem_reqstream_val   = 0;
  go                  = 0;
  base_addr_in        = base_addr;
  size_in             = size;

  //--------------------------------------------------------------------
  // STATE: XCFG
  //--------------------------------------------------------------------

  if ( state_reg == STATE_XCFG ) begin
    xcelreq_deq_rdy     = xcel_respstream_rdy;
    xcel_respstream_val = xcelreq_deq_val || done;
    if ( xcelreq_deq_val && (xcelreq_deq_msg.type_ == `XcelReqMsg_TYPE_WRITE) ) begin
      if ( xcelreq_deq_msg.addr == 0 )
        go = 1;
      else if ( xcelreq_deq_msg.addr == 1 )
        base_addr_in = xcelreq_deq_msg.data;
      else if ( xcelreq_deq_msg.addr == 2 )
        size_in = xcelreq_deq_msg.data;
        xcel_respstream_msg_raw.type_ = `XcelRespMsg_TYPE_WRITE;
        xcel_respstream_msg_raw.data  = 0;
    end
    else if ( done ) begin
        xcel_respstream_msg_raw.type_  = `XcelRespMsg_TYPE_READ;
        xcel_respstream_msg_raw.data   = 1;
    end
  end

  //--------------------------------------------------------------------
  // STATE: CALC
  //--------------------------------------------------------------------

  else if ( state_reg == STATE_CALC )
  begin
    if ((idx <= size - 1) && (idx - num_written <= num_cell - 1)) begin
      if (mem_reqstream_msg_raw.addr != addr_last) mem_reqstream_val = 1;
      else if (!memreq_sent) mem_reqstream_val = 1;
      else mem_reqstream_val = 0;
    end
    else mem_reqstream_val = 0;
    mem_reqstream_msg_raw.type_  = `VC_MEM_REQ_MSG_TYPE_READ;
    mem_reqstream_msg_raw.opaque = 0;
    mem_reqstream_msg_raw.addr   = base_addr + (idx << 2);
    mem_reqstream_msg_raw.len    = 0;
    mem_reqstream_msg_raw.data   = 0;

    in = memresp_deq_msg.data;
    memresp_deq_rdy = 1;
  end

  //--------------------------------------------------------------------
  // STATE: WRITE
  //--------------------------------------------------------------------

  else if ( state_reg == STATE_WRITE) begin
    mem_reqstream_val            = 1;
    mem_reqstream_msg_raw.type_  = `VC_MEM_REQ_MSG_TYPE_WRITE;
    mem_reqstream_msg_raw.opaque = 0;
    mem_reqstream_msg_raw.addr   = base_addr + ((temp - count) << 2);
    mem_reqstream_msg_raw.len    = 32;
    mem_reqstream_msg_raw.data   = out[temp - count];
  end

  //--------------------------------------------------------------------
  // STATE: READ
  //--------------------------------------------------------------------

  else if ( state_reg == STATE_READ) begin
    if (state_last == STATE_CALC && !memreq_sent_first) begin
      mem_reqstream_msg_raw.addr = base_addr + ((num_written - 1) << 2);
      mem_reqstream_val          = 1;
    end
    else if (state_last == STATE_CALC && memreq_sent_first && read_count == 32'b0)
      mem_reqstream_val = 0;
    else begin
      mem_reqstream_msg_raw.addr  = base_addr + ((num_written - 1 - read_count) << 2);
      if (read_count != num_written) begin
        if (mem_reqstream_msg_raw.addr != addr_last) mem_reqstream_val = 1;
        else if (!memreq_sent) mem_reqstream_val = 1;
        else mem_reqstream_val = 0;
      end
    end 

    mem_reqstream_msg_raw.type_  = `VC_MEM_REQ_MSG_TYPE_READ;
    mem_reqstream_msg_raw.opaque = 0;
    mem_reqstream_msg_raw.len    = 0;
    mem_reqstream_msg_raw.data   = 0;

    in = memresp_deq_msg.data;
    memresp_deq_rdy = 1;
  end

  //--------------------------------------------------------------------
  // STATE: MERGE
  //--------------------------------------------------------------------

  else if (state_reg == STATE_MERGE) begin
    mem_reqstream_val = 1;
    mem_reqstream_msg_raw.type_  = `VC_MEM_REQ_MSG_TYPE_WRITE;
    mem_reqstream_msg_raw.addr   = base_addr + ((num_written + count + 1 - read_count) << 2);
    mem_reqstream_msg_raw.opaque = 0;
    mem_reqstream_msg_raw.len    = 32;
    mem_reqstream_msg_raw.data   = merge_data;
    in = memresp_deq_msg.data;
  end
  end

  //----------------------------------------------------------------------
  // Line Tracing
  //----------------------------------------------------------------------

  `ifndef SYNTHESIS

  vc_XcelReqMsgTrace xcel_reqstream_msg_trace
  (
    .clk (clk),
    .reset (reset),
    .val   (xcel_reqstream_val),
    .rdy   (xcel_reqstream_rdy),
    .msg   (xcel_reqstream_msg)
  );

  vc_XcelRespMsgTrace xcel_respstream_msg_trace
  (
    .clk (clk),
    .reset (reset),
    .val   (xcel_respstream_val),
    .rdy   (xcel_respstream_rdy),
    .msg   (xcel_respstream_msg)
  );

  logic [`VC_TRACE_NBITS-1:0] str;
  `VC_TRACE_BEGIN
  begin
    xcel_reqstream_msg_trace.line_trace( trace_str );

    vc_trace.append_str( trace_str, "(" );

    $sformat( str, "%x", state_reg );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", mem_reqstream_msg.addr );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", mem_reqstream_msg.data );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", mem_respstream_msg.data );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", go );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", xcel_respstream_rdy );
    vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ", " );

    $sformat( str, "%x", done );
    vc_trace.append_str( trace_str, str );

    // vc_trace.append_str( trace_str, ", " );

    // $sformat( str, "%x", read_count );
    // vc_trace.append_str( trace_str, str );

    // vc_trace.append_str( trace_str, ", " );

    // $sformat( str, "%x", mem_respstream_val );
    // vc_trace.append_str( trace_str, str );

    // vc_trace.append_str( trace_str, ", " );

    // $sformat( str, "%x", mem_reqstream_rdy );
    // vc_trace.append_str( trace_str, str );

    vc_trace.append_str( trace_str, ")" );

    xcel_respstream_msg_trace.line_trace( trace_str );
  end
  `VC_TRACE_END

  `endif /* SYNTHESIS */

endmodule

`endif /* LAB2_XCEL_SORT_XCEL_V */
