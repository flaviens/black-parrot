/**
 *  bp_nonsynth_nbf_loader.v
 *
 */

`include "bp_common_defines.svh"
`include "bp_top_defines.svh"

module bp_nonsynth_nbf_loader
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)
   , parameter io_data_width_p = dword_width_gp

   , parameter nbf_filename_p = "prog.nbf"
   )
  (input                                            clk_i
   , input                                          reset_i

   , input [lce_id_width_p-1:0]                     lce_id_i
   , input [did_width_p-1:0]                        did_i

   , output logic [mem_header_width_lp-1:0]         io_cmd_header_o
   , output logic                                   io_cmd_header_v_o
   , input                                          io_cmd_header_ready_and_i
   , output logic                                   io_cmd_has_data_o
   , output logic [io_data_width_p-1:0]             io_cmd_data_o
   , output logic                                   io_cmd_data_v_o
   , input                                          io_cmd_data_ready_and_i
   , output logic                                   io_cmd_last_o

   , input  [mem_header_width_lp-1:0]               io_resp_header_i
   , input                                          io_resp_header_v_i
   , output logic                                   io_resp_header_ready_and_o
   , input                                          io_resp_has_data_i
   , input  [io_data_width_p-1:0]                   io_resp_data_i
   , input                                          io_resp_data_v_i
   , output logic                                   io_resp_data_ready_and_o
   , input                                          io_resp_last_i

   , output logic                                   done_o
   );

  `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);
  `bp_cast_o(bp_bedrock_mem_header_s, io_cmd_header);
  `bp_cast_i(bp_bedrock_mem_header_s, io_resp_header);

  enum logic [2:0] { e_reset, e_send, e_fence, e_done} state_n, state_r;
  wire is_reset    = (state_r == e_reset);
  wire is_send_nbf = (state_r == e_send);
  wire is_fence    = (state_r == e_fence);
  wire is_done     = (state_r == e_done);

  localparam max_nbf_index_lp = 2**26;
  localparam nbf_index_width_lp = `BSG_SAFE_CLOG2(max_nbf_index_lp);
  localparam nbf_data_width_lp = 64;
  localparam nbf_addr_width_lp = (paddr_width_p+3)/4*4;
  localparam nbf_opcode_width_lp = 8;
  typedef struct packed
  {
    logic [nbf_opcode_width_lp-1:0] opcode;
    logic [nbf_addr_width_lp-1:0] addr;
    logic [nbf_data_width_lp-1:0] data;
  } bp_nbf_s;

  // read nbf file
  bp_nbf_s nbf [max_nbf_index_lp-1:0];
  initial $readmemh(nbf_filename_p, nbf);

  bp_nbf_s curr_nbf;
  logic [nbf_index_width_lp-1:0] nbf_index_r, nbf_index_n;
  assign curr_nbf = nbf[nbf_index_r];

  wire is_fence_packet  = (curr_nbf.opcode == 8'hFE);
  wire is_finish_packet = (curr_nbf.opcode == 8'hFF);
  wire is_store_packet  = ~is_fence_packet & ~is_finish_packet;

  bp_bedrock_mem_header_s io_cmd_header_lo;
  logic io_cmd_v_lo, io_cmd_ready_and_li;
  logic [io_data_width_p-1:0] io_cmd_data_lo;
  bp_me_stream_pump_out
   #(.bp_params_p(bp_params_p)
     ,.stream_data_width_p(io_data_width_p)
     ,.block_width_p(cce_block_width_p)
     ,.payload_width_p(mem_payload_width_lp)
     ,.msg_stream_mask_p(mem_cmd_payload_mask_gp)
     ,.fsm_stream_mask_p(mem_cmd_payload_mask_gp)
     )
   pump_out
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
 
     ,.msg_header_o(io_cmd_header_cast_o)
     ,.msg_header_v_o(io_cmd_header_v_o)
     ,.msg_header_ready_and_i(io_cmd_header_ready_and_i)
     ,.msg_has_data_o(io_cmd_has_data_o)
     ,.msg_data_o(io_cmd_data_o)
     ,.msg_data_v_o(io_cmd_data_v_o)
     ,.msg_data_ready_and_i(io_cmd_data_ready_and_i)
     ,.msg_last_o(io_cmd_last_o)
 
     ,.fsm_base_header_i(io_cmd_header_lo)
     ,.fsm_data_i(io_cmd_data_lo)
     ,.fsm_v_i(io_cmd_v_lo)
     ,.fsm_ready_and_o(io_cmd_ready_and_li)
     ,.fsm_cnt_o()
     ,.fsm_new_o()
     ,.fsm_last_o()
     );

  wire next_nbf = (is_send_nbf && ((io_cmd_v_lo & io_cmd_ready_and_li) || is_fence_packet || is_finish_packet));
  bsg_counter_clear_up
   #(.max_val_p(max_nbf_index_lp-1), .init_val_p(0))
   nbf_counter
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.clear_i(1'b0)
     ,.up_i(next_nbf)
     ,.count_o(nbf_index_r)
     );

  logic [`BSG_WIDTH(io_noc_max_credits_p)-1:0] credit_count_lo;
  bsg_flow_counter
   #(.els_p(io_noc_max_credits_p))
   nbf_fc
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.v_i(io_cmd_header_v_o)
     ,.ready_i(io_cmd_header_ready_and_i)

     ,.yumi_i(io_resp_header_v_i & io_resp_header_ready_and_o)
     ,.count_o(credit_count_lo)
     );
  wire credits_full_lo = (credit_count_lo == io_noc_max_credits_p);
  wire credits_empty_lo = (credit_count_lo == '0);
  assign io_resp_header_ready_and_o = 1'b1;
  assign io_resp_data_ready_and_o = 1'b1;

  localparam sel_width_lp = `BSG_SAFE_CLOG2(nbf_data_width_lp>>3);
  localparam size_width_lp = `BSG_SAFE_CLOG2(sel_width_lp);
  logic [io_data_width_p-1:0] test;
  bsg_bus_pack
   #(.in_width_p(nbf_data_width_lp), .out_width_p(io_data_width_p))
   cmd_bus_pack
    (.data_i(curr_nbf.data)
     ,.sel_i('0) // We are aligned
     ,.size_i(io_cmd_header_lo.size[0+:size_width_lp])
     ,.data_o(io_cmd_data_lo)
     );

  always_comb
    begin
      io_cmd_header_lo = '0;
      io_cmd_header_lo.payload.lce_id = lce_id_i;
      io_cmd_header_lo.payload.did = did_i;
      io_cmd_header_lo.addr = curr_nbf.addr;
      io_cmd_header_lo.msg_type.mem = curr_nbf.opcode[5] ? e_bedrock_mem_uc_rd : e_bedrock_mem_uc_wr;
      io_cmd_header_lo.subop = e_bedrock_store;
      case (curr_nbf.opcode[1:0])
        2'b00: io_cmd_header_lo.size = e_bedrock_msg_size_1;
        2'b01: io_cmd_header_lo.size = e_bedrock_msg_size_2;
        2'b10: io_cmd_header_lo.size = e_bedrock_msg_size_4;
        2'b11: io_cmd_header_lo.size = e_bedrock_msg_size_8;
        default: io_cmd_header_lo.size = e_bedrock_msg_size_4;
      endcase
    end

  assign io_cmd_v_lo = ~credits_full_lo & is_send_nbf & ~is_fence_packet & ~is_finish_packet;

  always_comb
    unique casez (state_r)
      e_reset       : state_n = reset_i ? e_reset : e_send;
      e_send        : state_n = is_fence_packet
                                ? e_fence
                                : is_finish_packet
                                  ? e_done
                                  : e_send;
      e_fence       : state_n = credits_empty_lo ? e_send : e_fence;
      e_done        : state_n = e_done;
      default : state_n = e_reset;
    endcase
  assign done_o = is_done;

  //synopsys sync_set_reset "reset_i"
  always_ff @(posedge clk_i)
    if (reset_i)
      state_r <= e_reset;
    else
      state_r <= state_n;

  //synopsys translate_off
  always_ff @(negedge clk_i)
    begin
      if (state_r != e_done && state_n == e_done)
        $display("NBF loader done!");
    end
  //synopsys translate_on


  if (nbf_data_width_lp != dword_width_gp)
    $error("NBF data width must be same as dword_width_gp");
  if (io_data_width_p < nbf_data_width_lp)
    $error("NBF IO data width must be as large as NBF data width");

endmodule

