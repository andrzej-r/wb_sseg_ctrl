//
// Wishbone wrapper for seven-segment LED display controller
//
// Copyright (C) 2015 Andrzej <ndrwrdck@gmail.com>
//
// Redistribution and use in source and non-source forms, with or without
// modification, are permitted provided that the following conditions are met:
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//    * Redistributions in non-source form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
// THIS WORK IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// WORK, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// Register map (assuming 32b access):
// 00 [1]          - IRQ mask. 1 - IRQ enabled.
//    [0]          - Enable display controller
// 04 [15:0]       - Clock divider for multiplexing.
//                   To obtain a scanning rate further divide by 256 (for PWM) and by n_digits.
// 08 [7:0]        - Brightness control. 0 - minimum brightness (non-zero), 255 - maximum.
//                   Logarithmic progression (percepted).
// 0c [0]          - (Write) Write '1' to clear IRQ flag.
// 0c [0]          - (Read) IRQ Flag.
// 20 [n_segs-1:0] - First digit (or column)
// 24 [n_segs-1:0] - Second digit (or column)
// ...
//
// Example of instantiation:
//   // open-drain output with logic inversion
//   wire [7:0]            sseg_disp_seg_oe;
//   generate
//      for (i = 0; i < 8; i = i+1) begin: sseg_disp_seg_tris
//         assign sseg_disp_seg_o [i] = sseg_disp_seg_oe[i] ? 1'b0 : 1'bz;
//      end
//   endgenerate
//
//   wire [7:0] sseg_disp_seg_sel;
//   assign sseg_disp_an_o = ~sseg_disp_seg_sel;
//
//   wb_sseg_ctrl
//     #(
//       .n_digits   (8),
//`ifdef SIM
//       .def_clk_div(4) // speed up simulation time
//`else
//       .def_clk_div(128)
//`endif
//       )
//   sseg_ctrl
//     (
//      .wb_clk_i       (wb_clk),
//      .wb_rst_i       (wb_rst),
//      .async_rst_i    (async_rst),
//      // Wishbone slave interface
//      .wb_adr_i       (wb_m2s_sseg_ctrl_adr[5:2]),
//      .wb_dat_i       (wb_m2s_sseg_ctrl_dat),
//      .wb_sel_i       (wb_m2s_sseg_ctrl_sel),
//      .wb_we_i        (wb_m2s_sseg_ctrl_we),
//      .wb_cyc_i       (wb_m2s_sseg_ctrl_cyc),
//      .wb_stb_i       (wb_m2s_sseg_ctrl_stb),
//      .wb_cti_i       (wb_m2s_sseg_ctrl_cti),
//      .wb_bte_i       (wb_m2s_sseg_ctrl_bte),
//      .wb_dat_o       (wb_s2m_sseg_ctrl_dat),
//      .wb_ack_o       (wb_s2m_sseg_ctrl_ack),
//      .wb_err_o       (wb_s2m_sseg_ctrl_err),
//      .wb_rty_o       (wb_s2m_sseg_ctrl_rty),
//      // display i/f
//      .seg_o          (sseg_disp_seg_oe),
//      .seg_sel_o      (sseg_disp_seg_sel),
//      // frame sync irq
//      .irq_o          (sseg_didp_irq)
//      );


module wb_sseg_ctrl
  #(
    parameter n_digits    =    8,
    parameter n_segs      =    8,
    parameter def_clk_div =  128,
    //parameter def_clk_div =    4,
    parameter dw          =   16,
    parameter aw          =    4
    )
   (
    input                  wb_clk_i,
    input                  wb_rst_i,
    input                  async_rst_i,

    // Wishbone Interface
    input  [aw-1:0]        wb_adr_i,
    input  [dw-1:0]        wb_dat_i,
    input  [3:0]           wb_sel_i,
    input                  wb_we_i,
    input                  wb_cyc_i,
    input                  wb_stb_i,
    input  [2:0]           wb_cti_i,
    input  [1:0]           wb_bte_i,
    output reg [dw-1:0]    wb_dat_o,
    output reg             wb_ack_o,
    output                 wb_err_o,
    output                 wb_rty_o,

    // display i/f
    output [n_segs-1:0]    seg_o,
    output [n_digits-1:0]  seg_sel_o,

    // frame sync irq (end of the sweep)
    output                 irq_o
    );

   wire sync;

   // address decoder
   reg [2**aw-1:0]  sel;
   integer          i;
   always @(*)
     begin
        sel = {2**aw{1'b0}};
        for (i = 0; i < 2**aw; i = i + 1)
          if (wb_adr_i == i)
            sel[i] = 1'b1;
     end

   // enable register
   reg enable_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       enable_reg <= 1'b0;
     else if (wb_rst_i)
       enable_reg <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[0])
       enable_reg <= wb_dat_i[0];

   // mask IRQ register
   reg IRQ_mask_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       IRQ_mask_reg <= 1'b0;
     else if (wb_rst_i)
       IRQ_mask_reg <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[0])
       IRQ_mask_reg <= wb_dat_i[1];

   // clock divider register
   reg [15:0] clk_div_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       clk_div_reg <= def_clk_div;
     else if (wb_rst_i)
       clk_div_reg <= def_clk_div;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[1])
       clk_div_reg <= wb_dat_i[15:0];

   // brightness register
   reg [7:0] brightness_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       brightness_reg <= 8'hff;
     else if (wb_rst_i)
       brightness_reg <= 8'hff;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[2])
       brightness_reg <= wb_dat_i[7:0];

   // data to display
   reg [n_digits*n_segs-1:0] segments_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       segments_reg <= 0;
     else if (wb_rst_i)
       segments_reg <= 0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i)
       for (i = 0; i < n_digits; i = i + 1)
         if (sel[i+8])
           segments_reg[n_segs*(i+1)-1 -: n_segs] <= wb_dat_i[n_segs-1:0];

   // IRQ flag
   // write '1' to clear it
   reg IRQ_flag_reg;
   always @(posedge wb_clk_i or posedge async_rst_i)
     if (async_rst_i)
       IRQ_flag_reg <= 1'b0;
     else if (wb_rst_i)
       IRQ_flag_reg <= 1'b0;
     else if (wb_cyc_i & wb_stb_i & wb_we_i & sel[3] & wb_dat_i[0])
       IRQ_flag_reg <= 1'b0;
     else if (sync)
       IRQ_flag_reg <= 1'b1;

   assign irq_o = IRQ_flag_reg & IRQ_mask_reg;

   // read back register values
   always @(posedge wb_clk_i)
     if (wb_rst_i)
       wb_dat_o <= 32'b0;
     else if (wb_cyc_i)
       begin
          wb_dat_o <= 0;
          if (sel[0]) wb_dat_o[1:0]  <= {IRQ_mask_reg, enable_reg};
          if (sel[1]) wb_dat_o[15:0] <= clk_div_reg;
          if (sel[2]) wb_dat_o[7:0]  <= brightness_reg;
          if (sel[3]) wb_dat_o[0]    <= IRQ_flag_reg;
          for (i = 0; i < n_digits; i = i + 1)
            if (sel[i+8])
              wb_dat_o[n_segs-1:0] <= segments_reg[n_segs*(i+1)-1 -: n_segs];
       end

   // Ack generation
   always @(posedge wb_clk_i)
     if (wb_rst_i)
       wb_ack_o <= 0;
     else if (wb_ack_o)
       wb_ack_o <= 0;
     else if (wb_cyc_i & wb_stb_i & !wb_ack_o)
       wb_ack_o <= 1;

   assign wb_err_o = 0;
   assign wb_rty_o = 0;

   // instantiate the controller
   sseg_ctrl #(.n_digits(n_digits)) ctrl
     (
      .clk_i          (wb_clk_i),
      .rst_i          (wb_rst_i),
      .async_rst_i    (async_rst_i),
      // config registers
      .enable_i       (enable_reg),
      .clk_div_i      (clk_div_reg),
      .brightness_i   (brightness_reg),
      .segments_i     (segments_reg),
      // display i/f
      .seg_o          (seg_o),
      .seg_sel_o      (seg_sel_o),
      // sync irq
      .sync_o         (sync)
      );

endmodule
