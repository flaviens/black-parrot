/*
 * bp_fe_top.v
 */

`include "bp_common_defines.svh"
`include "bp_fe_defines.svh"

module bp_fe_top
 import bp_fe_pkg::*;
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_core_if_widths(vaddr_width_p, paddr_width_p, asid_width_p, branch_metadata_fwd_width_p)
   `declare_bp_cache_engine_if_widths(paddr_width_p, ctag_width_p, icache_sets_p, icache_assoc_p, dword_width_gp, icache_block_width_p, icache_fill_width_p, icache)

   , localparam cfg_bus_width_lp = `bp_cfg_bus_width(vaddr_width_p, hio_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p)
   )
  (input                                              clk_i
   , input                                            reset_i

   , input [cfg_bus_width_lp-1:0]                     cfg_bus_i

   , input [fe_cmd_width_lp-1:0]                      fe_cmd_i
   , input                                            fe_cmd_v_i
   , output                                           fe_cmd_yumi_o

   , output [fe_queue_width_lp-1:0]                   fe_queue_o
   , output                                           fe_queue_v_o
   , input                                            fe_queue_ready_i

   , output logic [icache_req_width_lp-1:0]           cache_req_o
   , output logic                                     cache_req_v_o
   , input                                            cache_req_ready_and_i
   , input                                            cache_req_busy_i
   , output logic [icache_req_metadata_width_lp-1:0]  cache_req_metadata_o
   , output logic                                     cache_req_metadata_v_o
   , input                                            cache_req_critical_tag_i
   , input                                            cache_req_critical_data_i
   , input                                            cache_req_complete_i
   , input                                            cache_req_credits_full_i
   , input                                            cache_req_credits_empty_i

   , input [icache_data_mem_pkt_width_lp-1:0]         data_mem_pkt_i
   , input                                            data_mem_pkt_v_i
   , output logic                                     data_mem_pkt_yumi_o
   , output logic [icache_block_width_p-1:0]          data_mem_o

   , input [icache_tag_mem_pkt_width_lp-1:0]          tag_mem_pkt_i
   , input                                            tag_mem_pkt_v_i
   , output logic                                     tag_mem_pkt_yumi_o
   , output logic [icache_tag_info_width_lp-1:0]      tag_mem_o

   , input [icache_stat_mem_pkt_width_lp-1:0]         stat_mem_pkt_i
   , input                                            stat_mem_pkt_v_i
   , output logic                                     stat_mem_pkt_yumi_o
   , output logic [icache_stat_info_width_lp-1:0]     stat_mem_o
   );

  `declare_bp_core_if(vaddr_width_p, paddr_width_p, asid_width_p, branch_metadata_fwd_width_p);
  `declare_bp_cfg_bus_s(vaddr_width_p, hio_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p);
  `declare_bp_fe_branch_metadata_fwd_s(btb_tag_width_p, btb_idx_width_p, bht_idx_width_p, ghist_width_p, bht_row_width_p);
  `bp_cast_i(bp_fe_cmd_s, fe_cmd);
  `bp_cast_o(bp_fe_queue_s, fe_queue);
  `bp_cast_i(bp_cfg_bus_s, cfg_bus);

  // FSM
  enum logic [1:0] {e_reset, e_wait, e_run, e_complex} state_n, state_r;

  // Decoded state signals
  wire is_reset    = (state_r == e_reset);
  wire is_wait     = (state_r == e_wait);
  wire is_run      = (state_r == e_run);
  wire is_complex  = (state_r == e_complex);

  wire state_reset_v          = (is_reset | is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_state_reset);
  wire pc_redirect_v          = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_pc_redirection);
  wire icache_fill_response_v = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode inside {e_op_icache_fill_restart, e_op_icache_fill_resume});
  wire wait_v                 = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_wait);

  wire itlb_fill_response_v   = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode inside {e_op_itlb_fill_restart, e_op_itlb_fill_resume});
  wire icache_fence_v         = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_icache_fence);
  wire itlb_fence_v           = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_itlb_fence);

  wire attaboy_v              = (is_wait | is_run) & fe_cmd_v_i & (fe_cmd_cast_i.opcode == e_op_attaboy);

  wire br_miss_v = pc_redirect_v
    & (fe_cmd_cast_i.operands.pc_redirect_operands.subopcode == e_subop_branch_mispredict);
  wire br_miss_taken = br_miss_v
    & (fe_cmd_cast_i.operands.pc_redirect_operands.misprediction_reason == e_incorrect_pred_taken);
  wire br_miss_ntaken = br_miss_v
    & (fe_cmd_cast_i.operands.pc_redirect_operands.misprediction_reason == e_incorrect_pred_ntaken);
  wire br_miss_nonbr = br_miss_v
    & (fe_cmd_cast_i.operands.pc_redirect_operands.misprediction_reason == e_not_a_branch);

  wire trap_v        = pc_redirect_v & (fe_cmd_cast_i.operands.pc_redirect_operands.subopcode == e_subop_trap);
  wire eret_v        = pc_redirect_v & (fe_cmd_cast_i.operands.pc_redirect_operands.subopcode == e_subop_eret);
  wire translation_v = pc_redirect_v & (fe_cmd_cast_i.operands.pc_redirect_operands.subopcode == e_subop_translation_switch);

  wire cmd_nonattaboy_v = fe_cmd_v_i & (fe_cmd_cast_i.opcode != e_op_attaboy);
  wire cmd_complex_v    = fe_cmd_v_i & (itlb_fill_response_v | icache_fence_v | itlb_fence_v);
  wire cmd_immediate_v  = fe_cmd_v_i & (state_reset_v | pc_redirect_v | icache_fill_response_v);

  logic [rv64_priv_width_gp-1:0] shadow_priv_n, shadow_priv_r;
  wire shadow_priv_w = state_reset_v | trap_v | eret_v;
  assign shadow_priv_n = fe_cmd_cast_i.operands.pc_redirect_operands.priv;
  bsg_dff_reset_en_bypass
   #(.width_p(rv64_priv_width_gp))
   shadow_priv_reg
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.en_i(shadow_priv_w)

     ,.data_i(shadow_priv_n)
     ,.data_o(shadow_priv_r)
     );

  logic shadow_translation_en_n, shadow_translation_en_r;
  wire shadow_translation_en_w = state_reset_v | trap_v | eret_v | translation_v;
  assign shadow_translation_en_n = fe_cmd_cast_i.operands.pc_redirect_operands.translation_en;
  bsg_dff_reset_en_bypass
   #(.width_p(1))
   shadow_translation_en_reg
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.en_i(shadow_translation_en_w)
     ,.data_i(shadow_translation_en_n)
     ,.data_o(shadow_translation_en_r)
     );

  logic pc_gen_init_done_lo;
  logic redirect_v_li;
  logic [vaddr_width_p-1:0] redirect_pc_li;
  logic redirect_br_v_li, redirect_br_taken_li, redirect_br_ntaken_li, redirect_br_nonbr_li;
  bp_fe_branch_metadata_fwd_s redirect_br_metadata_fwd_li;
  bp_fe_branch_metadata_fwd_s attaboy_br_metadata_fwd_li;
  logic attaboy_v_li, attaboy_yumi_lo, attaboy_taken_li, attaboy_ntaken_li;
  logic [vaddr_width_p-1:0] attaboy_pc_li;
  logic [instr_width_gp-1:0] fetch_li;
  logic [vaddr_width_p-1:0] fetch_pc_lo;
  logic fetch_instr_v_li, fetch_exception_v_li;
  bp_fe_branch_metadata_fwd_s fetch_br_metadata_fwd_lo;
  logic [vaddr_width_p-1:0] next_pc_lo;
  logic next_pc_yumi_li;
  logic ovr_lo;
  logic icache_tv_we;
  bp_fe_pc_gen
   #(.bp_params_p(bp_params_p))
   pc_gen
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.init_done_o(pc_gen_init_done_lo)

     ,.redirect_v_i(redirect_v_li)
     ,.redirect_pc_i(redirect_pc_li)
     ,.redirect_br_v_i(redirect_br_v_li)
     ,.redirect_br_metadata_fwd_i(redirect_br_metadata_fwd_li)
     ,.redirect_br_taken_i(redirect_br_taken_li)
     ,.redirect_br_ntaken_i(redirect_br_ntaken_li)
     ,.redirect_br_nonbr_i(redirect_br_nonbr_li)

     ,.next_pc_o(next_pc_lo)
     ,.next_pc_yumi_i(next_pc_yumi_li)

     ,.ovr_o(ovr_lo)
     ,.if2_we_i(icache_tv_we)

     ,.fetch_i(fetch_li)
     ,.fetch_instr_v_i(fetch_instr_v_li)
     ,.fetch_exception_v_i(fetch_exception_v_li)
     ,.fetch_br_metadata_fwd_o(fetch_br_metadata_fwd_lo)
     ,.fetch_pc_o(fetch_pc_lo)

     ,.attaboy_pc_i(attaboy_pc_li)
     ,.attaboy_br_metadata_fwd_i(attaboy_br_metadata_fwd_li)
     ,.attaboy_taken_i(attaboy_taken_li)
     ,.attaboy_ntaken_i(attaboy_ntaken_li)
     ,.attaboy_v_i(attaboy_v_li)
     ,.attaboy_yumi_o(attaboy_yumi_lo)
     );

  // Commands which should not redirect will not be acknowledged until we go back to running
  assign redirect_v_li               = ~attaboy_v_li & fe_cmd_yumi_o;
  assign redirect_pc_li              = fe_cmd_cast_i.npc;
  assign redirect_br_v_li            = br_miss_v;
  assign redirect_br_taken_li        = br_miss_taken;
  assign redirect_br_ntaken_li       = br_miss_ntaken;
  assign redirect_br_nonbr_li        = br_miss_nonbr;
  assign redirect_br_metadata_fwd_li = fe_cmd_cast_i.operands.pc_redirect_operands.branch_metadata_fwd;

  assign attaboy_v_li               = attaboy_v;
  assign attaboy_pc_li              = fe_cmd_cast_i.npc;
  assign attaboy_taken_li           = attaboy_v &  fe_cmd_cast_i.operands.attaboy.taken;
  assign attaboy_ntaken_li          = attaboy_v & ~fe_cmd_cast_i.operands.attaboy.taken;
  assign attaboy_br_metadata_fwd_li = fe_cmd_cast_i.operands.attaboy.branch_metadata_fwd;

  logic instr_access_fault_v, instr_page_fault_v;
  logic ptag_v_li, ptag_uncached_li, ptag_nonidem_li, ptag_dram_li, ptag_miss_li;
  logic [ptag_width_p-1:0] ptag_li;

  bp_pte_leaf_s w_tlb_entry_li;
  wire [vtag_width_p-1:0] w_vtag_li = fe_cmd_cast_i.npc[vaddr_width_p-1-:vtag_width_p];
  assign w_tlb_entry_li = fe_cmd_cast_i.operands.itlb_fill_response.pte_leaf;

  wire [dword_width_gp-1:0] r_eaddr_li = `BSG_SIGN_EXTEND(next_pc_lo, dword_width_gp);
  wire [1:0] r_size_li = 2'b10;
  logic itlb_r_v_li;
  bp_mmu
   #(.bp_params_p(bp_params_p)
     ,.tlb_els_4k_p(itlb_els_4k_p)
     ,.tlb_els_1g_p(itlb_els_1g_p)
     )
   immu
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.flush_i(itlb_fence_v)
     ,.priv_mode_i(shadow_priv_r)
     ,.trans_en_i(shadow_translation_en_r)
     // Supervisor use of user memory is always disabled for immu
     ,.sum_i('0)
     // Immu does not handle dcache loads
     ,.mxr_i('0)
     ,.uncached_mode_i((cfg_bus_cast_i.icache_mode == e_lce_mode_uncached))
     ,.nonspec_mode_i((cfg_bus_cast_i.icache_mode == e_lce_mode_nonspec))
     ,.hio_mask_i(cfg_bus_cast_i.hio_mask)

     ,.w_v_i(itlb_fill_response_v)
     ,.w_vtag_i(w_vtag_li)
     ,.w_entry_i(w_tlb_entry_li)

     ,.r_v_i(itlb_r_v_li)
     ,.r_instr_i(1'b1)
     ,.r_load_i('0)
     ,.r_store_i('0)
     ,.r_eaddr_i(r_eaddr_li)
     ,.r_size_i(r_size_li)

     ,.r_v_o(ptag_v_li)
     ,.r_ptag_o(ptag_li)
     ,.r_instr_miss_o(ptag_miss_li)
     ,.r_load_miss_o()
     ,.r_store_miss_o()
     ,.r_uncached_o(ptag_uncached_li)
     ,.r_nonidem_o(ptag_nonidem_li)
     ,.r_dram_o(ptag_dram_li)
     ,.r_instr_misaligned_o()
     ,.r_load_misaligned_o()
     ,.r_store_misaligned_o()
     ,.r_instr_access_fault_o(instr_access_fault_v)
     ,.r_load_access_fault_o()
     ,.r_store_access_fault_o()
     ,.r_instr_page_fault_o(instr_page_fault_v)
     ,.r_load_page_fault_o()
     ,.r_store_page_fault_o()
     );

  `declare_bp_fe_icache_pkt_s(vaddr_width_p);
  bp_fe_icache_pkt_s icache_pkt;
  assign icache_pkt = '{vaddr: next_pc_lo
                        ,op  : icache_fence_v ? e_icache_fencei : icache_fill_response_v ? e_icache_fill : e_icache_fetch
                        };
  // TODO: Should only ack icache fence when icache_ready
  // TODO: Add back Icache fence 
  logic [instr_width_gp-1:0] icache_data_lo;
  logic icache_data_v_lo, icache_spec_v_lo;
  logic icache_poison_req, icache_poison_tl;
  logic icache_v_li, icache_yumi_lo;
  bp_fe_icache
   #(.bp_params_p(bp_params_p))
   icache
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.cfg_bus_i(cfg_bus_i)

     ,.icache_pkt_i(icache_pkt)
     ,.v_i(icache_v_li)
     ,.force_i(cmd_nonattaboy_v)
     ,.yumi_o(icache_yumi_lo)
     ,.poison_req_i(icache_poison_req)

     ,.ptag_i(ptag_li)
     ,.ptag_v_i(ptag_v_li)
     ,.ptag_uncached_i(ptag_uncached_li)
     ,.ptag_nonidem_i(ptag_nonidem_li)
     ,.ptag_dram_i(ptag_dram_li)
     ,.poison_tl_i(icache_poison_tl)
     ,.tv_we_o(icache_tv_we)

     ,.data_o(icache_data_lo)
     ,.data_v_o(icache_data_v_lo)
     ,.spec_v_o(icache_spec_v_lo)
     ,.yumi_i(fe_queue_ready_i & (icache_data_v_lo | icache_spec_v_lo))

     ,.cache_req_o(cache_req_o)
     ,.cache_req_v_o(cache_req_v_o)
     ,.cache_req_ready_and_i(cache_req_ready_and_i)
     ,.cache_req_busy_i(cache_req_busy_i)
     ,.cache_req_metadata_o(cache_req_metadata_o)
     ,.cache_req_metadata_v_o(cache_req_metadata_v_o)
     ,.cache_req_critical_tag_i(cache_req_critical_tag_i)
     ,.cache_req_critical_data_i(cache_req_critical_data_i)
     ,.cache_req_complete_i(cache_req_complete_i)
     ,.cache_req_credits_full_i(cache_req_credits_full_i)
     ,.cache_req_credits_empty_i(cache_req_credits_empty_i)

     ,.data_mem_pkt_i(data_mem_pkt_i)
     ,.data_mem_pkt_v_i(data_mem_pkt_v_i)
     ,.data_mem_pkt_yumi_o(data_mem_pkt_yumi_o)
     ,.data_mem_o(data_mem_o)

     ,.tag_mem_pkt_i(tag_mem_pkt_i)
     ,.tag_mem_pkt_v_i(tag_mem_pkt_v_i)
     ,.tag_mem_pkt_yumi_o(tag_mem_pkt_yumi_o)
     ,.tag_mem_o(tag_mem_o)

     ,.stat_mem_pkt_v_i(stat_mem_pkt_v_i)
     ,.stat_mem_pkt_i(stat_mem_pkt_i)
     ,.stat_mem_pkt_yumi_o(stat_mem_pkt_yumi_o)
     ,.stat_mem_o(stat_mem_o)
     );

  logic itlb_miss_r, instr_access_fault_r, instr_page_fault_r;
  bsg_dff_reset_en
   #(.width_p(3))
   fault_reg
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.en_i(icache_tv_we)

     ,.data_i({ptag_miss_li, instr_access_fault_v, instr_page_fault_v})
     ,.data_o({itlb_miss_r, instr_access_fault_r, instr_page_fault_r})
     );

  wire fe_exception_v = (instr_access_fault_r | instr_page_fault_r | itlb_miss_r | icache_spec_v_lo);
  wire fe_instr_v     = icache_data_v_lo;
  assign fe_queue_v_o = fe_instr_v | fe_exception_v;

  // TODO: add to FSM poison 
  // The logic is to poison req and TL on exception going out.
  // For immediate CMD coming in, we want to poison TL
  // For complex CMD coming in, we want to poison TV
  // When complex CMD is being processed (fence) we don't want to poison the thing in TL
  // Or this will cause issues. So probably when transitioning to complex we poison, but when in
  // complex we don't poison 
  assign icache_poison_req = fe_exception_v;
  assign icache_poison_tl =
    ovr_lo
    | fe_exception_v
    | (fe_cmd_yumi_o & cmd_immediate_v)
    | (~is_complex & cmd_complex_v);

  assign fetch_instr_v_li     = fe_queue_ready_i & fe_queue_v_o & fe_instr_v;
  assign fetch_exception_v_li = fe_queue_ready_i & fe_queue_v_o & fe_exception_v;
  assign fetch_li             = icache_data_lo;

  always_comb
    if (fe_exception_v)
      begin
        fe_queue_cast_o = '0;
        fe_queue_cast_o.pc       = fetch_pc_lo;
        fe_queue_cast_o.msg_type = itlb_miss_r
                                   ? e_itlb_miss
                                     : instr_page_fault_r
                                       ? e_instr_page_fault
                                       : instr_access_fault_r
                                         ? e_instr_access_fault
                                           : e_icache_miss;
        fe_queue_cast_o.branch_metadata_fwd = fetch_br_metadata_fwd_lo;
        // TODO: Partially fetched exceptions
        fe_queue_cast_o.partial_v = 1'b0;
      end
    else
      begin
        fe_queue_cast_o = '0;
        fe_queue_cast_o.msg_type            = e_instr_fetch;
        fe_queue_cast_o.pc                  = fetch_pc_lo;
        fe_queue_cast_o.instr               = fetch_li;
        fe_queue_cast_o.branch_metadata_fwd = fetch_br_metadata_fwd_lo;
        // TODO: Partially fetched exceptions
        fe_queue_cast_o.partial_v = 1'b0;
      end

  // Controlling state machine
  always_comb
    begin
      state_n = state_r;
      icache_v_li = 1'b0;

      case (state_r)
        e_reset:
          begin
            icache_v_li = pc_gen_init_done_lo;
            itlb_r_v_li = icache_v_li;
            next_pc_yumi_li = icache_yumi_lo;
            state_n = (next_pc_yumi_li & state_reset_v) ? e_run : e_reset;
          end
        e_wait:
          begin
            icache_v_li = cmd_immediate_v;
            itlb_r_v_li = icache_v_li;
            next_pc_yumi_li = icache_yumi_lo;
            state_n = next_pc_yumi_li ? e_run : cmd_complex_v ? e_complex : e_wait;
          end
        e_complex:
          begin
            icache_v_li = fe_cmd_v_i;
            itlb_r_v_li = icache_v_li;
            next_pc_yumi_li = icache_yumi_lo;
            state_n = next_pc_yumi_li ? e_run : e_complex;
          end
        e_run:
          begin
            icache_v_li = ~cmd_complex_v || icache_fence_v;
            itlb_r_v_li = icache_v_li;
            next_pc_yumi_li = icache_yumi_lo & ~icache_fence_v;
            state_n = ((icache_fence_v & icache_yumi_lo) || cmd_complex_v) ? e_complex :
fetch_exception_v_li ? e_wait : e_run;
          end

        default: begin end
      endcase
    end

  assign fe_cmd_yumi_o = (cmd_nonattaboy_v & next_pc_yumi_li) || attaboy_yumi_lo;

  // synopsys sync_set_reset "reset_i"
  always_ff @(posedge clk_i)
    if (reset_i)
        state_r <= e_reset;
    else
      begin
        state_r <= state_n;
      end

endmodule

