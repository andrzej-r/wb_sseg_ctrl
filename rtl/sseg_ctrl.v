//
// Seven-segment LED display controller
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

module sseg_ctrl
  #(
    parameter n_digits    = 8,
    parameter n_segs      = 8
    )
   (
    input                        clk_i,
    input                        rst_i,
    input                        async_rst_i,

    // config registers
    input                        enable_i,
    input  [15:0]                clk_div_i,
    input  [7:0]                 brightness_i,
    input  [n_digits*n_segs-1:0] segments_i,

    // display i/f
    output [n_segs-1:0]          seg_o,
    output [n_digits-1:0]        seg_sel_o,

    // sync irq (end of the sweep)
    output                       sync_o
    );

   // time-base strobe generator
   reg [15:0] cnt_strobe_reg;
   wire strobe = (enable_i & cnt_strobe_reg == 16'b0);

   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       cnt_strobe_reg <= 16'b0;
     else if (rst_i | ~enable_i)
       cnt_strobe_reg <= 16'b0;
     else if (strobe)
       cnt_strobe_reg <= clk_div_i;
     else
       cnt_strobe_reg <= cnt_strobe_reg - 16'd1;

   // digit strobe generator
   reg [7:0] cnt_strobe_dig_reg;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       cnt_strobe_dig_reg <= 8'b0;
     else if (rst_i | ~enable_i)
       cnt_strobe_dig_reg <= 8'b0;
     else if (strobe)
       cnt_strobe_dig_reg <= cnt_strobe_dig_reg - 8'd1;

   wire strobe_dig = (strobe & cnt_strobe_dig_reg == 8'hff);

   // pwm reg
   reg pwm_reg;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       pwm_reg <= 1'b0;
     else if (rst_i | ~enable_i)
       pwm_reg <= 1'b0;
     else if (strobe)
       pwm_reg <= (cnt_strobe_dig_reg <= brightness_i);

   // frame strobe generator
   reg [4:0] cnt_strobe_frame_reg;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       cnt_strobe_frame_reg <= 5'b0;
     else if (rst_i | ~enable_i)
       cnt_strobe_frame_reg <= 5'b0;
     else if (strobe_dig &cnt_strobe_frame_reg == 5'b0)
       cnt_strobe_frame_reg <= n_digits - 1;
     else if (strobe_dig)
       cnt_strobe_frame_reg <= cnt_strobe_frame_reg - 5'd1;

   wire strobe_frame = (strobe_dig & cnt_strobe_frame_reg == 5'b0);

   // multiplex digits
   wire [n_digits-1:0] seg_sel_reg_new;
   assign seg_sel_reg_new = {strobe_frame, seg_sel_reg[n_digits-1:1]};

   reg [n_digits-1:0] seg_sel_reg;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       seg_sel_reg <= 0;
     else if (rst_i | ~enable_i)
       seg_sel_reg <= 0;
     else if (strobe_dig)
       seg_sel_reg <= seg_sel_reg_new;

   // output seg_sel
   assign seg_sel_o = seg_sel_reg; // & {n_digits{pwm_reg}};

   // segment value
   integer i;
   reg [n_segs-1:0] seg_reg;
   always @(posedge clk_i or posedge async_rst_i)
     if (async_rst_i)
       seg_reg <= 0;
     else if (rst_i | ~enable_i)
       seg_reg <= 0;
     else if (strobe_dig)
       for (i = 0; i < n_digits; i = i + 1)
         if (seg_sel_reg_new[i])
           seg_reg <= segments_i[n_segs*(n_digits-i)-1 -: n_segs];
           //seg_o <= segments_i[n_segs*(i+1)-1 -: n_segs];

   // output seg
   assign seg_o = seg_reg & {n_segs{pwm_reg}};

   assign sync_o = strobe_frame;

endmodule
