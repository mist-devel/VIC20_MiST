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

assign LED = ~ioctl_download & ~led_disk & cass_motor;

`include "build_id.v"

localparam TAP_MEM_START = 22'h20000;

localparam CONF_STR =
{
    "VIC20;PRGCRTTAP;",
    "S,D64,Mount Disk;",
    "TC,Play/Stop TAP;",
    "OD,Tape sound,Off,On;",
    "O3,Video,PAL,NTSC;",
    "O2,CRT with load address,Yes,No;",
    "OAB,Scanlines,Off,25%,50%,75%;",
    "O45,Enable 8K+ Expansion,Off,8K,16K,24K;",
    "O6,Enable 3K Expansion,Off,On;",
    "O78,Enable 8k ROM,Off,RO,RW;",
    "O9,Audio Filter,On,Off;",
    "T0,Reset;",
    "T1,Reset with cart unload;",
    "V,v1.0.",`BUILD_DATE
};


////////////////////   CLOCKS   ///////////////////
wire clk_sys;
wire clk_32;
wire clk_1541 = clk_32;
reg clk8m;
reg clk16m;
wire pll_locked;
reg clk_ref; //sync sdram to during prg downloading
reg  reset;
reg  c1541_reset;
reg cart_unload;
reg force_reset;

wire       pll_reconfig_busy;
wire       pll_areset;
wire       pll_configupdate;
wire       pll_scanclk;
wire       pll_scanclkena;
wire       pll_scandata;
wire       pll_scandataout;
wire       pll_scandone;
wire       pll_reconfig_reset;
wire [7:0] pll_rom_address;
wire       pll_rom_q;
wire       pll_write_from_rom;
wire       pll_write_rom_ena;
wire       pll_reconfig;
wire       q_reconfig_ntsc;
wire       q_reconfig_pal;

rom_reconfig_pal rom_reconfig_pal
(
    .address(pll_rom_address),
    .clock(clk_32),
    .rden(pll_write_rom_ena),
    .q(q_reconfig_pal)
);

rom_reconfig_ntsc rom_reconfig_ntsc
(
    .address(pll_rom_address),
    .clock(clk_32),
    .rden(pll_write_rom_ena),
    .q(q_reconfig_ntsc)
);

assign pll_rom_q = st_ntsc ? q_reconfig_ntsc : q_reconfig_pal;

pll_reconfig pll_reconfig_inst
(
    .busy(pll_reconfig_busy),
    .clock(clk_32),
    .counter_param(0),
    .counter_type(0),
    .data_in(0),
    .pll_areset(pll_areset),
    .pll_areset_in(0),
    .pll_configupdate(pll_configupdate),
    .pll_scanclk(pll_scanclk),
    .pll_scanclkena(pll_scanclkena),
    .pll_scandata(pll_scandata),
    .pll_scandataout(pll_scandataout),
    .pll_scandone(pll_scandone),
    .read_param(0),
    .reconfig(pll_reconfig),
    .reset(pll_reconfig_reset),
    .reset_rom_address(0),
    .rom_address_out(pll_rom_address),
    .rom_data_in(pll_rom_q),
    .write_from_rom(pll_write_from_rom),
    .write_param(0),
    .write_rom_ena(pll_write_rom_ena)
);

pll_vic20 pll_vic20
(
    .inclk0(CLOCK_27[0]),
    .c0(clk_sys),  //35.48 MHz PAL, 28.63 MHz NTSC
    .areset(pll_areset),
    .scanclk(pll_scanclk),
    .scandata(pll_scandata),
    .scanclkena(pll_scanclkena),
    .configupdate(pll_configupdate),
    .scandataout(pll_scandataout),
    .scandone(pll_scandone),
    .locked(pll_locked)
);

always @(posedge clk_32) begin
    reg ntsc_d, ntsc_d2, ntsc_d3;
    reg [1:0] pll_reconfig_state = 0;
    reg [9:0] pll_reconfig_timeout;

    ntsc_d <= st_ntsc;
    ntsc_d2 <= ntsc_d;
    pll_write_from_rom <= 0;
    pll_reconfig <= 0;
    pll_reconfig_reset <= 0;
    case (pll_reconfig_state)
    2'b00:
    begin
        ntsc_d3 <= ntsc_d2;
        if (ntsc_d2 ^ ntsc_d3) begin
            pll_write_from_rom <= 1;
            pll_reconfig_state <= 2'b01;
        end
    end
    2'b01: pll_reconfig_state <= 2'b10;
    2'b10:
        if (~pll_reconfig_busy) begin
            pll_reconfig <= 1;
            pll_reconfig_state <= 2'b11;
            pll_reconfig_timeout <= 10'd1000;
        end
    2'b11:
    begin
        pll_reconfig_timeout <= pll_reconfig_timeout - 1'd1;
        if (pll_reconfig_timeout == 10'd1) begin
            // pll_reconfig stuck in busy state
            pll_reconfig_reset <= 1;
            pll_reconfig_state <= 2'b00;
        end
        if (~pll_reconfig & ~pll_reconfig_busy) pll_reconfig_state <= 2'b00;
    end
    default: ;
    endcase
end

pll27 pll
(
    .inclk0(CLOCK_27[0]),
    .c0(clk_32) //32 MHz
);

always @(posedge clk_sys) begin
    reg [4:0] sys_count;
    clk8m <= !sys_count[1:0];
    clk16m <= sys_count[0];
    clk_ref <= !sys_count;
    sys_count <= sys_count + 1'd1;
    
    reset <= st_reset | st_cart_unload | buttons[1] | force_reset | ~pll_locked;
    cart_unload <= 0;
    if (st_cart_unload | buttons[1]) cart_unload <= 1;
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
wire        ypbpr;

// status word wires (9 is unused)
wire [31:0] status;
wire        st_reset               = status[0];
wire        st_cart_unload         = status[1];
wire        st_crt_no_load_address = status[2];
wire        st_ntsc                = status[3];
wire  [1:0] st_ram_expansion       = status[5:4];
wire        st_3k_expansion        = status[6];
wire  [1:0] st_8k_rom              = status[8:7];
wire        st_audio_filter        = ~status[9];
wire  [1:0] st_scanlines           = status[11:10];
wire        st_tap_play_btn        = status[12];
wire        st_tape_sound          = status[13];

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
    .clk_sd(clk_1541),
    .SPI_SS_IO(CONF_DATA0),
    .SPI_CLK(SPI_SCK),
    .SPI_MOSI(SPI_DI),
    .SPI_MISO(SPI_DO),
//    .SPI_SS2(SPI_SS2),
	
    .conf_str(CONF_STR),

    .status(status),
    .scandoubler_disable(scandoubler_disable),
    .ypbpr(ypbpr),
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

wire  [7:0] col_in;
wire  [7:0] row_out;
wire  [7:0] row_in;
wire  [7:0] col_out;
wire        restore_key;

keyboard keyboard
(
    .reset(reset),
    .clk_sys(clk_sys),
    .ps2_kbd_clk(ps2Clk),
    .ps2_kbd_data(ps2Data),
    .col_in(col_in),
    .row_out(row_out),
    .row_in(row_in),
    .col_out(col_out),
    .restore_key(restore_key)
);

wire  [7:0] vic20_joy = joystick_0 | joystick_1;

vic20 VIC20
(
    .I_SYSCLK(clk_sys),
    .I_SYSCLK_EN(clk8m & ~ioctl_download),
    .I_RESET(reset),
    .I_PAL(~st_ntsc),

    .I_JOY(~{vic20_joy[0],vic20_joy[1],vic20_joy[2],vic20_joy[3]}),
    .I_FIRE(~vic20_joy[4]),
    .O_VIDEO_R(R_O),
    .O_VIDEO_G(G_O),
    .O_VIDEO_B(B_O),
    .O_HSYNC(HS_O),
    .O_VSYNC(VS_O),
//    .O_DE     => core_blankn_s,

	.atn_o(vic20_iec_atn_o),
	.clk_o(vic20_iec_clk_o),
	.data_o(vic20_iec_data_o),
	.clk_i(c1541_iec_clk_o),
    .data_i(c1541_iec_data_o),

    .O_ROW_IN(row_in),
    .I_COL_OUT(col_out),
    .O_COL_IN(col_in),
    .I_ROW_OUT(row_out),
    .I_RESTORE_OUT(restore_key),

    .I_CART_EN(|st_8k_rom),  // at $A000(8k)
    .I_CART_RO(st_8k_rom != 2'd2),
    .I_RAM_EXT({&st_ram_expansion, st_ram_expansion[1], |st_ram_expansion, st_3k_expansion}), //at $6000(8k),$4000(8k),$2000(8k),$0400(3k)

    .O_CASS_WRITE(cass_write),
    .I_CASS_READ(cass_read),
    .O_CASS_MOTOR(cass_motor),
    .I_CASS_SENSE(cass_sense),

    .O_AUDIO(vic_audio),
    .O_AUDIO_FILTERED(vic_audio_filtered),

    .o_extmem_sel(sdram_en),
    .o_extmem_r_wn(sdram_wr_n),
    .o_extmem_addr(sdram_vic20_a),
    .i_extmem_data(sdram_out),
    .o_extmem_data(sdram_in),
    
    .o_p2_h(p2_h),

    // -- ROM setup bus
    .CONF_WR(ioctl_internal_memory_wr & ioctl_ram_wr),
    .CONF_AI(ioctl_target_addr[15:0]),
    .CONF_DI(ioctl_reg_inject_state ? ioctl_reg_data : ioctl_dout)
);

//////////////////   MEMORY   //////////////////
assign SDRAM_CLK = clk_sys;

wire  [7:0] sdram_in;
wire  [7:0] sdram_out;
wire [15:0] sdram_vic20_a;
wire [22:0] sdram_a;
wire        sdram_wr_n;
wire        sdram_en;
reg         sdram_access;
wire        p2_h;

always_comb begin
    casex ({prg_download | tap_download, p2_h})
    'b01 : sdram_a = {9'b0, cart_unload ? 16'ha004 : sdram_vic20_a};
    'b00 : sdram_a = tap_play_addr;
    'b1X : sdram_a = ioctl_target_addr;
    endcase
end

sdram ram
(
    .*,
    .clkref(ioctl_download ? ioctl_wr : p2_h),
    .init(~pll_locked),
    .clk(clk_sys),
    .bank(2'b00),
    .dout(sdram_out),
    .din ((prg_download | tap_download) ? ioctl_dout : sdram_in),
    .addr(sdram_a),
    .we((sdram_en & ~sdram_wr_n) || (prg_download && !ioctl_internal_memory_wr && ioctl_ram_wr) || (tap_download && ioctl_ram_wr) || cart_unload),
    .oe(p2_h ? sdram_en & sdram_wr_n : tap_sdram_oe)
);

//////////////////  PRG/ROM/TAP LOAD //////////////
wire        ioctl_wr;
wire        ioctl_ram_wr;
wire [24:0] ioctl_addr;
wire  [7:0] ioctl_dout;
wire        ioctl_download;
wire  [7:0] ioctl_index;
wire        rom_download = ioctl_download && !ioctl_index;
wire        prg_download = ioctl_download && (ioctl_index == 8'h01 || ioctl_index == 8'h41);
wire        tap_download = ioctl_download && ioctl_index == 8'h81;
reg   [4:0] ioctl_reg_inject_state = 0;
wire [22:0] ioctl_target_addr;
reg  [22:0] ioctl_tap_addr;
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
    casex ({tap_download, rom_download, ioctl_addr[15:13]})
        'bX1_00X: ioctl_target_addr = {7'h0, 2'b00, ioctl_addr[13:0]}; //1541
        'bX1_010: ioctl_target_addr = {7'h0, 3'b111, ioctl_addr[12:0]}; //kernal pal
        'bX1_011: ioctl_target_addr = {7'h0, 3'b111, ioctl_addr[12:0]}; //kernal ntsc
        'bX1_100: ioctl_target_addr = {7'h0, 3'b110, ioctl_addr[12:0]}; //basic
        'bX1_101: ioctl_target_addr = {7'h0, 4'b1000, ioctl_addr[11:0]}; //character
        'b00_XXX: ioctl_target_addr = {7'h0, ioctl_reg_inject_state ? ioctl_reg_addr : ioctl_prg_addr};
        'b1X_XXX: ioctl_target_addr = ioctl_tap_addr;
        default: ioctl_target_addr = 0;
    endcase;
end

wire ioctl_internal_memory_wr = 
    (rom_download && ioctl_target_addr[15:14]) ||
    (prg_download && (ioctl_target_addr[15:10] == 6'b000000 ||
                      ioctl_target_addr[15:11] == 5'b00010 ||
                      ioctl_target_addr[15:11] == 5'b00011 ||
                      ioctl_target_addr[15:10] == 6'b100101));

always @(posedge clk_sys) begin
    reg old_prg_download;
    reg auto_reset;

    force_reset <= 0;
    old_prg_download <= prg_download;
    ioctl_ram_wr <= 0;
    if (prg_download && ioctl_wr) begin
        if (~st_crt_no_load_address) begin //cart/prg loading with address in the first 2 bytes
            if (ioctl_addr == 16'h0000) ioctl_prg_addr[7:0] <= ioctl_dout; else
            if (ioctl_addr == 16'h0001) ioctl_prg_addr[15:8] <= ioctl_dout; else begin
                ioctl_ram_wr <= 1;
                if (ioctl_addr != 16'h0002) ioctl_prg_addr <= ioctl_prg_addr + 1'd1;
            end
        end else begin
            ioctl_prg_addr <= ioctl_addr ? ioctl_prg_addr + 1'd1 : 16'ha000; //load to $a000
            ioctl_ram_wr <= 1;
        end
        if (ioctl_prg_addr == 16'ha000) auto_reset <= 1;
    end
    if (tap_download && ioctl_wr) begin
        ioctl_tap_addr <= ioctl_addr ? ioctl_tap_addr + 1'd1 : TAP_MEM_START; //load tap to 20000
        ioctl_ram_wr <= 1;
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
        31: begin force_reset <= auto_reset; auto_reset <= 0; end
    endcase

    if (ioctl_reg_inject_state) ioctl_reg_inject_state <= ioctl_reg_inject_state + 1'd1;
end

//////////////////   TAPE   //////////////////

reg [21:0] tap_play_addr;
reg [21:0] tap_last_addr;
reg  [7:0] tap_data_in;
reg        tap_reset;
reg        tap_wrreq;
reg        tap_wrfull;
reg        tap_version;
reg        tap_sdram_oe;
wire       cass_read;
wire       cass_write;
wire       cass_motor;
wire       cass_sense;

always @(posedge clk_sys) begin
    reg p2_hD;

    if (reset) begin
        tap_play_addr <= TAP_MEM_START;
        tap_last_addr <= TAP_MEM_START;
        tap_sdram_oe <= 0;
        tap_reset <= 1;
    end else begin
        tap_reset <= 0;
        if (tap_download) begin
            tap_play_addr <= TAP_MEM_START;
            tap_last_addr <= ioctl_tap_addr;
            tap_reset <= 1;
            if (ioctl_addr == 24'h0C && ioctl_wr) begin
                tap_version <= ioctl_dout[0];
            end
        end
        p2_hD <= p2_h;
        tap_wrreq <= 0;
        if (p2_hD && !p2_h && !ioctl_download && tap_play_addr != tap_last_addr && !tap_wrfull) tap_sdram_oe <= 1;
        if (!p2_h && tap_sdram_oe) tap_data_in <= sdram_out;
        if (p2_h && !p2_hD && tap_sdram_oe) begin
            tap_wrreq <= 1;
            tap_sdram_oe <= 0;
            tap_play_addr <= tap_play_addr + 1'd1;
        end
    end
end

c1530 c1530
(
    .clk32(clk_sys),
    .restart_tape(tap_reset),
    .wav_mode(0),
    .tap_version(tap_version),
    .host_tap_in(tap_data_in),
    .host_tap_wrreq(tap_wrreq),
    .tap_fifo_wrfull(tap_wrfull),
    .tap_fifo_error(),
    .cass_read(cass_read),
    .cass_write(cass_write),
    .cass_motor(cass_motor),
    .cass_sense(cass_sense),
    .osd_play_stop_toggle(st_tap_play_btn),
    .ear_input(UART_RX)
);
//////////////////   AUDIO   //////////////////

wire [15:0] vic_audio, vic_audio_filtered;
wire [15:0] audio_sel = st_audio_filter ? vic_audio_filtered : vic_audio;
wire [15:0] cass_audio = { (~cass_read | (cass_write & ~cass_motor & ~cass_sense)), 12'd0 };  // silence cass_write when motor is off because bit is in common with keyboard
wire [15:0] audio_out = st_tape_sound ? audio_sel + cass_audio : audio_sel;

sigma_delta_dac #(15) dac_l
(
    .CLK(clk_sys),
    .RESET(reset),
    .DACin(audio_sel),
    .DACout(AUDIO_L)
);

sigma_delta_dac #(15) dac_r
(
    .CLK(clk_sys),
    .RESET(reset),
    .DACin(audio_out),
    .DACout(AUDIO_R)
);
//////////////////   VIDEO   //////////////////

wire  [3:0] R_O;
wire  [3:0] G_O;
wire  [3:0] B_O;
wire        HS_O;
wire        VS_O;
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
wire        csync_out = ~(~HS_O | ~VS_O);

// a minimig vga->scart cable expects a composite sync signal on the VGA_HS output.
// and VCC on VGA_VS (to switch into rgb mode)
assign      VGA_HS = (scandoubler_disable || ypbpr)? csync_out : SD_HS_O;
assign      VGA_VS = (scandoubler_disable || ypbpr)? 1'b1 : SD_VS_O;

scandoubler scandoubler
(
    .clk_sys(clk_sys),
    .scanlines(st_scanlines),
    .hs_in(HS_O),
    .vs_in(VS_O),
    .r_in(R_O),
    .g_in(G_O),
    .b_in(B_O),
    .hs_out(SD_HS_O),
    .vs_out(SD_VS_O),
    .r_out(SD_R_O),
    .g_out(SD_G_O),
    .b_out(SD_B_O)
);

wire [5:0] osd_r_o, osd_g_o, osd_b_o;

osd osd
(
    .clk_sys(clk_sys),
    .ce_pix(scandoubler_disable ? clk8m : clk16m),
    .SPI_DI(SPI_DI),
    .SPI_SCK(SPI_SCK),
    .SPI_SS3(SPI_SS3),
    .R_in(scandoubler_disable ? {R_O, 2'b00} : SD_R_O),
    .G_in(scandoubler_disable ? {G_O, 2'b00} : SD_G_O),
    .B_in(scandoubler_disable ? {B_O, 2'b00} : SD_B_O),
    .HSync(scandoubler_disable ? HS_O : SD_HS_O),
    .VSync(scandoubler_disable ? VS_O : SD_VS_O),
    .R_out(osd_r_o),
    .G_out(osd_g_o),
    .B_out(osd_b_o)
    );

wire [5:0] y, pb, pr;

rgb2ypbpr rgb2ypbpr 
(
	.red   ( osd_r_o ),
	.green ( osd_g_o ),
	.blue  ( osd_b_o ),
	.y     ( y       ),
	.pb    ( pb      ),
	.pr    ( pr      )
);
	 
assign VGA_R = ypbpr?pr:osd_r_o;
assign VGA_G = ypbpr? y:osd_g_o;
assign VGA_B = ypbpr?pb:osd_b_o;
	 
//////////////////   DISK   //////////////////

wire led_disk;
wire vic20_iec_atn_o;
wire vic20_iec_data_o;
wire vic20_iec_clk_o;

wire c1541_iec_atn_o;
wire c1541_iec_data_o;
wire c1541_iec_clk_o;

c1541_sd c1541_sd (
    .clk32 ( clk_1541 ),
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
