//============================================================================
// 
//  VIC20 replica for MiST
//  Copyright (C) 2018 György Szombathelyi
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//============================================================================

`default_nettype none

module vic20_mist
(
   input         CLOCK_27[0],   // Input clock 27 MHz

   output  [5:0] VGA_R,
   output  [5:0] VGA_G,
   output  [5:0] VGA_B,
   output        VGA_HS,
   output        VGA_VS,

   output        LED,

   output        AUDIO_L,
   output        AUDIO_R,

   input         UART_RX,

   input         SPI_SCK,
   output        SPI_DO,
   input         SPI_DI,
   input         SPI_SS2,
   input         SPI_SS3,
   input         CONF_DATA0,

   output [12:0] SDRAM_A,
   inout  [15:0] SDRAM_DQ,
   output        SDRAM_DQML,
   output        SDRAM_DQMH,
   output        SDRAM_nWE,
   output        SDRAM_nCAS,
   output        SDRAM_nRAS,
   output        SDRAM_nCS,
   output  [1:0] SDRAM_BA,
   output        SDRAM_CLK,
   output        SDRAM_CKE
);

assign LED = ~ioctl_download & ~led_disk;

`include "build_id.v"
localparam CONF_STR = 
{
	"VIC20;PRG;",
	"F1,CRT;",
	"S,D64,Mount Disk;",
	"O2,CRT with load address,No,Yes;",
	"OAB,Scanlines,Off,25%,50%,75%;",
	"O45,Enable 8K+ Expansion,Off,8K,16K,24K;",
	"O6,Enable 3K Expansion,Off,On;",
    "O78,Enable 8k ROM,Off,RO,RW;",
	"T0,Reset;",
	"V,v1.0.",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////
wire clk_sys;
reg clk8m;
reg clk16m;
wire pll_locked;
reg clk_ref; //sync sdram to during prg downloading
reg  reset;
reg  c1541_reset;

pll27 pll
(
    .inclk0(CLOCK_27[0]),
    .c0(clk_sys),
    .locked(pll_locked)
);

always @(posedge clk_sys) begin
    reg [4:0] sys_count;
    clk8m <= !sys_count[1:0];
    clk16m <= !sys_count[0];
    clk_ref <= !sys_count;
    sys_count <= sys_count + 1'd1;
    
    reset <= status[0] | buttons[1] | ~pll_locked;
    c1541_reset <= reset;
end

//////////////////   MIST ARM I/O   ///////////////////
wire        ps2Clk;
wire        ps2Data;

wire  [7:0] joystick_0;
wire  [7:0] joystick_1;
wire  [1:0] buttons;
wire  [1:0] switches;
wire        scandoubler_disable;
wire [31:0] status;

wire [31:0] sd_lba;
wire        sd_rd;
wire        sd_wr;
wire        sd_ack;
wire  [7:0] sd_dout;
wire        sd_dout_strobe;
wire        sd_din;
wire  [8:0] sd_buff_addr;
wire        img_mounted;

user_io #(.STRLEN($size(CONF_STR)>>3)) user_io
(
    .clk_sys(clk_sys),
    .clk_sd(clk_sys),
    .SPI_SS_IO(CONF_DATA0),
    .SPI_CLK(SPI_SCK),
    .SPI_MOSI(SPI_DI),
    .SPI_MISO(SPI_DO),
//    .SPI_SS2(SPI_SS2),
	
    .conf_str(CONF_STR),

    .status(status),
    .scandoubler_disable(scandoubler_disable),
    .buttons(buttons),
    .switches(switches),
    .joystick_0(joystick_0),
    .joystick_1(joystick_1),
    .ps2_kbd_clk(ps2Clk),
    .ps2_kbd_data(ps2Data),
    
    .sd_lba(sd_lba),
    .sd_rd(sd_rd),
    .sd_wr(sd_wr),
    .sd_ack(sd_ack),
    .sd_dout(sd_dout),
    .sd_dout_strobe(sd_dout_strobe),
    .sd_din(sd_din),
    .sd_buff_addr(sd_buff_addr),
    .sd_conf(0),
    .sd_sdhc(1),
    .img_mounted(img_mounted),

	// unused
    .ps2_key(),
    .ps2_mouse_clk(),
    .ps2_mouse_data(),
    .joystick_analog_0(),
    .joystick_analog_1(),
    .sd_ack_conf()
);

wire  [7:0] matrix_in;
wire  [7:0] matrix_out;
wire        restore_key;

keyboard keyboard
(
    .reset(reset),
    .clk_sys(clk_sys),
    .ps2_kbd_clk(ps2Clk),
    .ps2_kbd_data(ps2Data),
    .matrix_in(matrix_in),
    .matrix_out(matrix_out),
    .restore_key(restore_key)
);

wire  [7:0] vic20_joy = joystick_0 | joystick_1;

vic20 VIC20
(
    .I_SYSCLK(clk_sys),
    .I_SYSCLK_EN(clk8m & ~ioctl_download),
    .I_RESET(reset),

    .I_JOY(~{vic20_joy[0],vic20_joy[1],vic20_joy[2],vic20_joy[3]}),
    .I_FIRE(~vic20_joy[4]),
    .O_VIDEO_R(VGA_R_O),
    .O_VIDEO_G(VGA_G_O),
    .O_VIDEO_B(VGA_B_O),
    .O_HSYNC(VGA_HS_O),
    .O_VSYNC(VGA_VS_O),
//    .O_DE     => core_blankn_s,

	.atn_o(vic20_iec_atn_o),
	.clk_o(vic20_iec_clk_o),
	.data_o(vic20_iec_data_o),
	.clk_i(c1541_iec_clk_o),
    .data_i(c1541_iec_data_o),

    .O_MATRIX_IN(matrix_in),
    .I_MATRIX_OUT(matrix_out),
    .I_RESTORE_OUT(restore_key),

    .I_CART_EN(|status[8:7]),  // at $A000(8k)
    .I_CART_RO(status[8:7] != 2'd2),
    .I_RAM_EXT({&status[5:4], status[5], |status[5:4], status[6]}), //at $6000(8k),$4000(8k),$2000(8k),$0400(3k)

    .O_AUDIO(vic_audio),

    .o_extmem_sel(sdram_en),
    .o_extmem_r_wn(sdram_wr_n),
    .o_extmem_addr(sdram_a),
    .i_extmem_data(sdram_out),
    .o_extmem_data(sdram_in),
    
    .o_p2_h(p2_h),

    // -- ROM setup bus
    .CONF_WR(ioctl_internal_memory_wr & ioctl_ram_wr),
    .CONF_AI(ioctl_target_addr),
    .CONF_DI(ioctl_reg_inject_state ? ioctl_reg_data : ioctl_dout)
);

//////////////////   MEMORY   //////////////////
assign SDRAM_CLK = ~clk_sys;
wire  [7:0] sdram_in;
wire  [7:0] sdram_out;
wire [15:0] sdram_a;
wire        sdram_wr_n;
wire        sdram_en;
reg         sdram_access;
wire        p2_h;

sdram ram
(
    .*,
    .clkref(ioctl_download ? clk_ref : p2_h),
    .init(~pll_locked),
    .clk(clk_sys),
    .bank(2'b00),
    .dout(sdram_out),
    .din (prg_download ? ioctl_dout : sdram_in),
    .addr(prg_download ? ioctl_target_addr : {9'b0, sdram_a[15:0]}),
    .we((sdram_en & ~sdram_wr_n) || (prg_download && !ioctl_internal_memory_wr && ioctl_ram_wr)),
    .oe(sdram_en & sdram_wr_n)
);

//////////////////  PRG/ROM LOAD //////////////
wire        ioctl_wr;
wire        ioctl_ram_wr;
wire [15:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        rom_download = ioctl_download && !ioctl_index;
wire        prg_download = ioctl_download && ioctl_index[4:0] == 5'd1;
reg   [4:0] ioctl_reg_inject_state = 0;
wire [15:0] ioctl_target_addr;
reg  [15:0] ioctl_prg_addr;
reg  [15:0] ioctl_reg_addr;
reg         ioctl_reg_wr;
reg   [7:0] ioctl_reg_data;

data_io data_io (
    // SPI interface
    .sck ( SPI_SCK ),
    .ss  ( SPI_SS2 ),
    .sdi ( SPI_DI  ),
    // ram interface
    .clk   ( clk_sys ),
    .clkref( clk_ref  ),
    .downloading ( ioctl_download ),
    .index ( ioctl_index ),
    .wr    ( ioctl_wr ),
    .a     ( ioctl_addr ),
    .d     ( ioctl_dout )
);

always_comb begin
    casex ({rom_download, ioctl_addr[15:13]})
        'b1_00X: ioctl_target_addr = {2'b00, ioctl_addr[13:0]}; //1541
        'b1_010: ioctl_target_addr = {3'b111, ioctl_addr[12:0]}; //kernal
        'b1_011: ioctl_target_addr = {3'b110, ioctl_addr[12:0]}; //basic
        'b1_100: ioctl_target_addr = {4'b1000, ioctl_addr[11:0]}; //character
        'b0_XXX: ioctl_target_addr = ioctl_reg_inject_state ? ioctl_reg_addr : ioctl_prg_addr;
        default: ioctl_target_addr = 0;
    endcase;
end

wire ioctl_internal_memory_wr = 
    (rom_download && ioctl_target_addr[15:14]) ||
    (!rom_download && (ioctl_target_addr[15:10] == 6'b000000 ||
                       ioctl_target_addr[15:11] == 5'b00010 ||
                       ioctl_target_addr[15:11] == 5'b00011 ||
                       ioctl_target_addr[15:10] == 6'b100101));

always @(negedge clk_sys) begin
    reg old_prg_download;
    
    old_prg_download <= prg_download;
    ioctl_ram_wr <= 0;
    if (prg_download && ioctl_wr) begin
        if (ioctl_addr == 16'h0000) ioctl_prg_addr[7:0] <= ioctl_dout; else
        if (ioctl_addr == 16'h0001) ioctl_prg_addr[15:8] <= ioctl_dout; else begin
            ioctl_ram_wr <= 1;
            if (ioctl_addr != 16'h0002) ioctl_prg_addr <= ioctl_prg_addr + 1'd1;
        end
    end
    if (rom_download) ioctl_ram_wr <= ioctl_wr;
    
    //prg download ended, adjust registers
    if (old_prg_download & ~prg_download) ioctl_reg_inject_state <= 1;
    
    case (ioctl_reg_inject_state)
		 1: begin ioctl_reg_addr <= 16'h2d; ioctl_reg_data <= ioctl_prg_addr[7:0];  ioctl_ram_wr <= 1; end
		 3: begin ioctl_reg_addr <= 16'h2e; ioctl_reg_data <= ioctl_prg_addr[15:8]; ioctl_ram_wr <= 1; end
		 5: begin ioctl_reg_addr <= 16'h2f; ioctl_reg_data <= ioctl_prg_addr[7:0];  ioctl_ram_wr <= 1; end
		 7: begin ioctl_reg_addr <= 16'h30; ioctl_reg_data <= ioctl_prg_addr[15:8]; ioctl_ram_wr <= 1; end
		 9: begin ioctl_reg_addr <= 16'h31; ioctl_reg_data <= ioctl_prg_addr[7:0];  ioctl_ram_wr <= 1; end
		11: begin ioctl_reg_addr <= 16'h32; ioctl_reg_data <= ioctl_prg_addr[15:8]; ioctl_ram_wr <= 1; end
		13: begin ioctl_reg_addr <= 16'hae; ioctl_reg_data <= ioctl_prg_addr[7:0];  ioctl_ram_wr <= 1; end
		15: begin ioctl_reg_addr <= 16'haf; ioctl_reg_data <= ioctl_prg_addr[15:8]; ioctl_ram_wr <= 1; end
    endcase

    if (ioctl_reg_inject_state) ioctl_reg_inject_state <= ioctl_reg_inject_state + 1'd1;
end

//////////////////   AUDIO   //////////////////

wire [15:0] vic_audio;

sigma_delta_dac #(15) dac_l
(
    .CLK(clk_sys),
    .RESET(reset),
    .DACin(vic_audio),
    .DACout(AUDIO_L)
);

sigma_delta_dac #(15) dac_r
(
    .CLK(clk_sys),
    .RESET(reset),
    .DACin(vic_audio),
    .DACout(AUDIO_R)
);
//////////////////   VIDEO   //////////////////

wire  [3:0] VGA_R_O;
wire  [3:0] VGA_G_O;
wire  [3:0] VGA_B_O;
wire        VGA_HS_O;
wire        VGA_VS_O;
wire        SD_HS_O;
wire        SD_VS_O;
wire        osd_hs_in;
wire        osd_vs_in;

wire  [5:0] SD_R_O;
wire  [5:0] SD_G_O;
wire  [5:0] SD_B_O;
wire  [5:0] osd_r_in;
wire  [5:0] osd_g_in;
wire  [5:0] osd_b_in;

wire        vsync_out;
wire        hsync_out;
wire        csync_out = (hsync_out == vsync_out);

// a minimig vga->scart cable expects a composite sync signal on the VGA_HS output.
// and VCC on VGA_VS (to switch into rgb mode)
assign      VGA_HS = scandoubler_disable ? csync_out : hsync_out;
assign      VGA_VS = scandoubler_disable ? 1'b1 : vsync_out;

scandoubler scandoubler
(
    .clk_sys(clk_sys),
    .scanlines(status[11:10]),
    .hs_in(VGA_HS_O),
    .vs_in(VGA_VS_O),
    .r_in(VGA_R_O),
    .g_in(VGA_G_O),
    .b_in(VGA_B_O),
    .hs_out(SD_HS_O),
    .vs_out(SD_VS_O),
    .r_out(SD_R_O),
    .g_out(SD_G_O),
    .b_out(SD_B_O)
);

osd osd
(
    .clk_sys(clk_sys),
    .ce_pix(scandoubler_disable ? clk8m : clk16m),
    .sdi(SPI_DI),
    .sck(SPI_SCK),
    .ss(SPI_SS3),
    .red_in(scandoubler_disable ? {VGA_R_O, 2'b00} : SD_R_O),
    .green_in(scandoubler_disable ? {VGA_G_O, 2'b00} : SD_G_O),
    .blue_in(scandoubler_disable ? {VGA_B_O, 2'b00} : SD_B_O),
    .hs_in(scandoubler_disable ? VGA_HS_O : SD_HS_O),
    .vs_in(scandoubler_disable ? VGA_VS_O : SD_VS_O),
    .red_out(VGA_R),
    .green_out(VGA_G),
    .blue_out(VGA_B),
    .hs_out(hsync_out),
    .vs_out(vsync_out)
    );

//////////////////   DISK   //////////////////

wire led_disk;
wire vic20_iec_atn_o;
wire vic20_iec_data_o;
wire vic20_iec_clk_o;

wire c1541_iec_atn_o;
wire c1541_iec_data_o;
wire c1541_iec_clk_o;

c1541_sd c1541_sd (
    .clk32 ( clk_sys ),
    .reset ( c1541_reset ),

    .disk_change ( img_mounted ),
    .disk_num ( 10'd0 ), // always 0 on MiST, the image is selected by the OSD menu

	.iec_atn_i  ( vic20_iec_atn_o  ),
	.iec_data_i ( vic20_iec_data_o ),
	.iec_clk_i  ( vic20_iec_clk_o  ),
	.iec_data_o ( c1541_iec_data_o ),
    .iec_clk_o  ( c1541_iec_clk_o ),

    .sd_lba         ( sd_lba         ),
    .sd_rd          ( sd_rd          ),
    .sd_wr          ( sd_wr          ),
    .sd_ack         ( sd_ack         ),
    .sd_buff_din    ( sd_din         ),
    .sd_buff_dout   ( sd_dout        ),
    .sd_buff_wr     ( sd_dout_strobe ),
    .sd_buff_addr   ( sd_buff_addr   ),
    .led            ( led_disk       ),

    .c1541rom_clk   ( clk_sys         ),
    .c1541rom_addr  ( ioctl_addr[13:0]),
    .c1541rom_data  ( ioctl_dout      ),
    .c1541rom_wr    ( ioctl_wr & rom_download & !ioctl_addr[15:14] )
);

endmodule
