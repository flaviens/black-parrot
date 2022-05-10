`include "bp_common_defines.svh"
`include "bp_top_defines.svh"

// This accelerator is just a wrapper around a scratchpad memory. For now, the
//   of elements is hardcoded.

module bp_sacc_loopback
 import bp_common_pkg::*;
 import bp_be_pkg::*;
 import bp_me_pkg::*;
 #(parameter bp_params_e bp_params_p = e_bp_default_cfg
   `declare_bp_proc_params(bp_params_p)
   `declare_bp_bedrock_mem_if_widths(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p)
   , localparam cfg_bus_width_lp= `bp_cfg_bus_width(vaddr_width_p, hio_width_p, core_id_width_p, cce_id_width_p, lce_id_width_p)
   )
  (input                                        clk_i
   , input                                      reset_i

   , input [lce_id_width_p-1:0]                 lce_id_i

   , input [mem_header_width_lp-1:0]            io_cmd_header_i
   , input                                      io_cmd_header_v_i
   , output logic                               io_cmd_header_ready_and_o
   , input                                      io_cmd_has_data_i

   , input [acache_fill_width_p-1:0]            io_cmd_data_i
   , input                                      io_cmd_data_v_i
   , output logic                               io_cmd_data_ready_and_o
   , input                                      io_cmd_last_i

   , output logic [mem_header_width_lp-1:0]     io_resp_header_o
   , output logic                               io_resp_header_v_o
   , input                                      io_resp_header_ready_and_i

   , output logic [acache_fill_width_p-1:0]     io_resp_data_o
   , output logic                               io_resp_data_v_o
   , input                                      io_resp_data_ready_and_i
   , output logic                               io_resp_last_o
   );

  // CCE-IO interface is used for uncached requests-read/write memory mapped CSR
  `declare_bp_bedrock_mem_if(paddr_width_p, did_width_p, lce_id_width_p, lce_assoc_p);
  `declare_bp_memory_map(paddr_width_p, daddr_width_p);
  `bp_cast_i(bp_bedrock_mem_header_s, io_cmd_header);
  `bp_cast_o(bp_bedrock_mem_header_s, io_resp_header);

  localparam reg_els_lp = 2;

  // We use this just for the flow control
  // The address decoding doesn't easily work with the streaming accelerator region
  logic r_v_lo, w_v_lo;
  logic [paddr_width_p-1:0] addr_lo;
  logic [dword_width_gp-1:0] data_lo;
  logic [dword_width_gp-1:0] data_li;
  bp_me_bedrock_register
   #(.bp_params_p(bp_params_p)
     ,.els_p(1)
     ,.reg_addr_width_p(paddr_width_p)
     ,.base_addr_p({64'h????????????????})
     )
   register
    (.*
     // We ignore reads because these are all asynchronous registers
     ,.r_v_o(r_v_lo)
     ,.w_v_o(w_v_lo)
     ,.addr_o(addr_lo)
     ,.size_o()
     ,.data_o(data_lo)
     ,.data_i(data_li)
     );
  bp_local_addr_s  local_addr_li;
  bp_global_addr_s global_addr_li;
  assign local_addr_li = addr_lo;
   assign global_addr_li = addr_lo;

  wire spm_w_v_lo        = w_v_lo & (global_addr_li.hio == 1'b1);
  wire spm_r_v_lo        = r_v_lo & (global_addr_li.hio == 1'b1);
  wire csr_wr_cnt_w_v_lo = w_v_lo & (global_addr_li.hio == 1'b0) & (local_addr_li.addr == accel_wr_cnt_csr_idx_gp);
  wire csr_wr_cnt_r_v_lo = r_v_lo & (global_addr_li.hio == 1'b0) & (local_addr_li.addr == accel_wr_cnt_csr_idx_gp);

  logic [dword_width_gp-1:0] spm_write_cnt;
  bsg_counter_clear_up
   #(.max_val_p(0), .init_val_p(0), .ptr_width_lp(dword_width_gp))
   wr_counter
    (.clk_i(clk_i)
     ,.reset_i(reset_i)

     ,.clear_i(1'b0)
     ,.up_i(spm_w_v_lo)
     ,.count_o(spm_write_cnt)
     );

  localparam spm_els_lp          = 20;
  localparam spm_addr_width_lp   = `BSG_SAFE_CLOG2(spm_els_lp);
  localparam spm_offset_width_lp = `BSG_SAFE_CLOG2(dword_width_gp);

  // SPM
  logic [dword_width_gp-1:0] spm_data_lo;
  wire [spm_addr_width_lp-1:0] spm_addr_li = addr_lo >> spm_offset_width_lp;
  bsg_mem_1rw_sync
   #(.width_p(dword_width_gp), .els_p(spm_els_lp))
   accel_spm
    (.clk_i(clk_i)
     ,.reset_i(reset_i)
     ,.data_i(data_lo)
     ,.addr_i(spm_addr_li)
     ,.v_i(spm_r_v_lo | spm_w_v_lo)
     ,.w_i(spm_w_v_lo)
     ,.data_o(spm_data_lo)
     );

  logic spm_r_v_r, csr_wr_cnt_r_v_r;
  bsg_dff
   #(.width_p(2))
   r_v_reg
    (.clk_i(clk_i)
     ,.data_i({spm_r_v_lo, csr_wr_cnt_r_v_lo})
     ,.data_o({spm_r_v_r, csr_wr_cnt_r_v_r})
     );

  assign data_li = spm_r_v_r ? spm_data_lo : spm_write_cnt;

endmodule

