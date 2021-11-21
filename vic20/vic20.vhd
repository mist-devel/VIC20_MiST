--
-- A simulation model of VIC20 hardware
--
-- All rights reserved
-- (c) copyright 2003-2009 by MikeJ (Mike Johnson)
-- http://www.FPGAArcade.com - mikej <at> fpgaarcade <dot> com
-- (c) copyright 2011...2015 by WoS (Wolfgang Scherr)
-- http://www.pin4.at - WoS <at> pin4 <dot> at
--
-- $Id: vic20.vhd 2205 2017-08-04 19:28:32Z mikej $
--
----------------------------------------------------------------------------
--
-- Redistribution and use in source and synthezised forms, with or without
-- modification, are permitted provided that the following conditions are met:
--
-- Redistributions of source code must retain the above copyright notice,
-- this list of conditions and the following disclaimer.
--
-- Redistributions in synthesized form must reproduce the above copyright
-- notice, this list of conditions and the following disclaimer in the
-- documentation and/or other materials provided with the distribution.
--
-- Neither the name of the author nor the names of other contributors may
-- be used to endorse or promote products derived from this software without
-- specific prior written permission; any commercial use is forbidden as well.
--
-- This code must be run on Replay hardware only.
--
-- THIS CODE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
-- AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
-- THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
-- PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE
-- LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
-- CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
-- SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
-- INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
-- CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
-- ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
-- POSSIBILITY OF SUCH DAMAGE.
--
-- You are responsible for any legal issues arising from your use of this code.
--
-- The latest version of this file can be found at: www.fpgaarcade.com
--
-- Email vic20@fpgaarcade.com
--

library ieee ;
  use ieee.std_logic_1164.all ;
  use ieee.std_logic_unsigned.all;
  use ieee.numeric_std.all;

--library UNISIM;
--  use UNISIM.Vcomponents.all;

entity VIC20 is
  generic (
    I_EXTERNAL_ROM        : std_logic := '0'
  );
  port (
    --
    i_sysclk              : in    std_logic;  -- comes from CLK_A via DCM (divided by 4)
    i_sysclk_en           : in    std_logic;  -- 8.867236 MHz (PAL),
                                              -- 7.15909 (NTSC) enable signal
    i_reset               : in    std_logic;
    i_pal                 : in    std_logic;
    -- serial bus pins
    atn_o                 : out std_logic; -- open drain
    atn_i                 : in  std_logic;
    clk_o                 : out std_logic; -- open drain
    clk_i                 : in  std_logic;
    data_o                : out std_logic; -- open drain
    data_i                : in  std_logic;
    --
    i_joy                 : in    std_logic_vector(3 downto 0); -- 0 up, 1 down, 2 left,  3 right
    i_fire                : in    std_logic;                    -- all low active
    --
    i_cart_en             : in    std_logic; -- at $A000(8k)
    i_cart_ro             : in    std_logic; -- read-only cartridge if set
    i_ram_ext             : in    std_logic_vector(3 downto 0); -- at $6000(8k),$4000(8k),$2000(8k),$0400(3k)
    --
    o_video_r             : out   std_logic_vector(3 downto 0);
    o_video_g             : out   std_logic_vector(3 downto 0);
    o_video_b             : out   std_logic_vector(3 downto 0);
    o_hsync               : out   std_logic;
    o_vsync               : out   std_logic;
    o_de                  : out   std_logic;
    --
    o_row_in              : out   std_logic_vector(7 downto 0); -- take care, positive logic (1=>selected)
    i_col_out             : in    std_logic_vector(7 downto 0); -- take care, positive logic (1=>pressed)
    o_col_in              : out   std_logic_vector(7 downto 0); -- take care, positive logic (1=>selected)
    i_row_out             : in    std_logic_vector(7 downto 0); -- take care, positive logic (1=>pressed)
    i_restore_out         : in    std_logic;                    -- take care, positive logic (1=>pressed)
	 --
    o_cass_write          : out   std_logic;
    i_cass_read           : in    std_logic;
    o_cass_motor          : out   std_logic;
    i_cass_sense          : in    std_logic;
     --
    o_audio               : out   std_logic_vector(15 downto 0); -- runs at SYSCLK/SYSCLK_EN rate
    o_audio_filtered      : out   std_logic_vector(15 downto 0);
    -- back to system DRAM controller for external memory and cartridges, just map 1:1 to VIC memory
    o_extmem_sel          : out   std_logic;
    o_extmem_r_wn         : out   std_logic;
    o_extmem_addr         : out   std_logic_vector(15 downto 0);
    i_extmem_data         : in    std_logic_vector(7 downto 0);
    o_extmem_data         : out   std_logic_vector(7 downto 0);
    
    o_p2_h                : out   std_logic;
    
    -- you may leave this disconnected if not required
    debug                 : out   std_logic_vector(32 downto 0);  -- amount of channels of DLA (e.g. via aux_io)
    debugi                : in    std_logic_vector(2 downto 0);
    --configures "embedded" core memory
    conf_wr               : in    std_logic;
    conf_ai               : in    std_logic_vector(15 downto 0);
    conf_di               : in    std_logic_vector(7 downto 0)
    );
end;

-- PAL version runs with a 8,867,236 Hz Quartz which is divided by two for display, eight for CPU
-- NTSC version runs with a 14,318,18 Hz Quartz which is divided by 3.5 for display, fourteen for CPU

architecture RTL of VIC20 is
  -- selects CPU related debugging signals for external debug
  constant ext_dla_cpu_debug : boolean := false;
  -- selects CPU related debugging signals for internal debug
  constant int_dla_cpu_debug : boolean := false;

    -- default
    --constant K_OFFSET : std_logic_vector (4 downto 0) := "10000"; -- h position of screen to centre on your telly
    -- lunar lander is WAY off to the left
    --constant K_OFFSET : std_logic_vector (4 downto 0) := "11100"; -- h position of screen to centre on your telly

    -- disable 24k expansion version
    constant disable_24k : boolean := true;
    -- disable 16k expansion version
    constant disable_16k : boolean := false;
    -- 4k cartridge (otherwise 8k)
    constant cartridge_4k : boolean := true;

    -- system clock for audio filters
    constant sysclk_PAL  : integer := 35480000;
    constant sysclk_NTSC : integer := 28630000;

    signal reset_l            : std_logic;
    signal ena_4              : std_logic;
    signal reset_l_sampled    : std_logic;
    -- cpu
    signal c_ena              : std_logic;
    signal c_addr             : std_logic_vector(23 downto 0);
    signal c_din              : std_logic_vector(7 downto 0);
    signal c_din_s            : std_logic_vector(7 downto 0);
    signal c_dout             : std_logic_vector(7 downto 0);
    signal c_rw_l             : std_logic;
    signal c_irq_l            : std_logic;
    signal c_nmi_l            : std_logic;
    --
    signal io_sel_l           : std_logic_vector(3 downto 0);
    signal blk_sel_l          : std_logic_vector(7 downto 0);
    signal ram_sel_l          : std_logic_vector(7 downto 0);

    -- vic
    signal vic_addr           : std_logic_vector(13 downto 0);
    signal vic_oe_l           : std_logic;
    signal vic_dout           : std_logic_vector( 7 downto 0);
    signal vic_din            : std_logic_vector(11 downto 0);
    signal p2_h               : std_logic;
    signal p2_h_rise          : std_logic;
    signal p2_h_fall          : std_logic;
    signal ena_1mhz           : std_logic;
    signal via1_dout          : std_logic_vector( 7 downto 0);
    signal via2_dout          : std_logic_vector( 7 downto 0);
		
    signal vic_audio           : std_logic_vector( 5 downto 0);
    signal lp_output       : std_logic_vector(15 downto 0);
    signal lp_filtered     : std_logic_vector(15 downto 0);
	 signal filter_count : unsigned(5 downto 0);
	 signal filter_tick : std_logic;

    -- video system
    signal v_addr             : std_logic_vector(13 downto 0);
    signal v_data             : std_logic_vector( 7 downto 0);
    signal v_data_oe_l        : std_logic;
    signal v_data_read_mux    : std_logic_vector( 7 downto 0);
    signal v_data_read_muxr   : std_logic_vector( 7 downto 0);
    signal v_rw_l             : std_logic;
    signal col_ram_sel_l      : std_logic;

    -- ram
    signal ram0_dout          : std_logic_vector(7 downto 0);
    signal ram45_dout         : std_logic_vector(7 downto 0);
    signal ram67_dout         : std_logic_vector(7 downto 0);
    --
    signal col_ram_dout       : std_logic_vector(3 downto 0);

    -- rom
    signal char_rom_dout      : std_logic_vector(7 downto 0);
    signal basic_rom_dout     : std_logic_vector(7 downto 0);
    signal kernal_pal_dout    : std_logic_vector(7 downto 0);
    signal kernal_ntsc_dout   : std_logic_vector(7 downto 0);
    signal kernal_rom_dout    : std_logic_vector(7 downto 0);

    -- expansion
    signal expansion_din      : std_logic_vector(7 downto 0);
    signal expansion_nmi_l    : std_logic;
    signal expansion_irq_l    : std_logic;

    -- VIAs
    signal via1_nmi           : std_logic;
    signal via1_pa_in         : std_logic_vector(7 downto 0);
    signal via1_pa_out        : std_logic_vector(7 downto 0);

    signal via2_irq           : std_logic;

    signal cass_write         : std_logic;
    signal cass_read          : std_logic;
    signal cass_motor         : std_logic;
    signal cass_sw            : std_logic;

    signal keybd_col_out      : std_logic_vector(7 downto 0);
    signal keybd_col_out_s    : std_logic_vector(7 downto 0);
    signal keybd_col_in       : std_logic_vector(7 downto 0);
    signal keybd_col_oe       : std_logic_vector(7 downto 0);
    signal keybd_row_in       : std_logic_vector(7 downto 0);
    signal keybd_row_out      : std_logic_vector(7 downto 0);
    signal keybd_row_out_s    : std_logic_vector(7 downto 0);
    signal keybd_row_oe       : std_logic_vector(7 downto 0);
    signal keybd_restore      : std_logic;

    signal joy                : std_logic_vector(3 downto 0);
    signal light_pen          : std_logic;

    signal serial_srq_in      : std_logic;
    signal serial_atn_out_l   : std_logic;
    signal serial_atn_in      : std_logic; -- the vic does not listen to atn_in
    signal serial_clk_out_l   : std_logic;
    signal serial_clk_in      : std_logic;
    signal serial_data_out_l  : std_logic;
    signal serial_data_in     : std_logic;

    -- user port
    signal user_port_cb1_in   : std_logic;
    signal user_port_cb1_out  : std_logic;
    signal user_port_cb1_oe_l : std_logic;
    signal user_port_cb2_in   : std_logic;
    signal user_port_cb2_out  : std_logic;
    signal user_port_cb2_oe_l : std_logic;
    signal user_port_in       : std_logic_vector(7 downto 0);
    signal user_port_out      : std_logic_vector(7 downto 0);
    signal user_port_oe_l     : std_logic_vector(7 downto 0);
    -- misc
    signal sw_reg             : std_logic_vector(3 downto 0);
    signal extmem             : std_logic;

    signal video_r            : std_logic_vector(3 downto 0);
    signal video_g            : std_logic_vector(3 downto 0);
    signal video_b            : std_logic_vector(3 downto 0);
    signal hsync              : std_logic;
    signal vsync              : std_logic;
    signal csync              : std_logic;
    signal de                 : std_logic;
    
    signal kernal_pal_wr      : std_logic;
    signal kernal_ntsc_wr     : std_logic;
    signal kernal_pal_written : std_logic := '0';
    signal basic_wr           : std_logic;
    signal char_wr            : std_logic;

begin
  o_p2_h <= p2_h;

  -- <= c_rw_l;
  -- <= v_rw_l;
  expansion_nmi_l <= '1';
  expansion_irq_l <= '1';
  -- <= ram_sel_l;
  -- <= io_sel_l;
  -- <= reset_l_sampled;

  -- user port
  user_port_cb1_in <= '1';
  user_port_cb2_in <= '1';
  user_port_in <= x"FF";
  -- <= user_port_out
  -- <= user_port_out_oe_l

  -- tape
  O_CASS_WRITE <= cass_write;
  cass_read    <= I_CASS_READ;
  O_CASS_MOTOR <= cass_motor;
  cass_sw      <= I_CASS_SENSE;

  -- serial
  serial_srq_in <= '1';
  serial_clk_in <= clk_i and not serial_clk_out_l;
  serial_data_in <= data_i and not serial_data_out_l;
  serial_atn_in <= atn_i and not serial_atn_out_l;
  atn_o <= not serial_atn_out_l;
  clk_o <= not serial_clk_out_l;
  data_o <= not serial_data_out_l;

  -- joy
  joy <= i_joy;        -- 0 up, 1 down, 2 left,  3 right
  light_pen <= i_fire; -- also used for fire button
  --
  --
  --
  reset_l <= not i_reset;
  --
  u_clocks : entity work.VIC20_CLOCKS
    port map (
      I_SYSCLK          => i_sysclk,
      I_SYSCLK_EN       => i_sysclk_en,
      I_RESET_L         => reset_l,
      I_PAL             => i_pal,
      --
      O_ENA             => ena_4,
      O_RESET_L         => reset_l_sampled
      );

  c_ena <= ena_1mhz and ena_4; -- clk ena
  c_din_s <= c_dout when c_rw_l = '0' else c_din;

  cpu : entity work.T65
      port map (
          Mode    => "00",
          Res_n   => reset_l_sampled,
          Enable  => c_ena,
          Clk     => i_sysclk,
          Rdy     => '1',
          Abort_n => '1',
          IRQ_n   => c_irq_l,
          NMI_n   => c_nmi_l,
          SO_n    => '1',
          R_W_n   => c_rw_l,
          Sync    => open,
          EF      => open,
          MF      => open,
          XF      => open,
          ML_n    => open,
          VP_n    => open,
          VDA     => open,
          VPA     => open,
          A       => c_addr,
          DI      => c_din_s,
          DO      => c_dout
      );

  vic : entity work.M6561
    port map (
      I_CLK           => i_sysclk,
      I_ENA_4         => ena_4,
      I_RESET_L       => reset_l,
      O_ENA_1MHZ      => ena_1mhz,
      O_P2_H          => p2_h,
      O_P2_H_RISE     => p2_h_rise,
      O_P2_H_FALL     => p2_h_fall,

      I_RW_L          => v_rw_l,

      I_ADDR          => v_addr(13 downto 0),
      O_ADDR          => vic_addr(13 downto 0),

      I_DATA          => vic_din,
      O_DATA          => vic_dout,
      O_DATA_OE_L     => vic_oe_l,
      --
      O_AUDIO         => vic_audio,

      O_VIDEO_R       => video_r,
      O_VIDEO_G       => video_g,
      O_VIDEO_B       => video_b,

      O_HSYNC         => hsync,
      O_VSYNC         => vsync,
      O_COMP_SYNC_L   => csync,
      O_DE            => de,
      --
      I_PAL           => i_pal,
      --
      I_LIGHT_PEN     => light_pen,
      I_POTX          => '0',
      I_POTY          => '0'
      );
	 
	 -- audio filters
	
	 -- Filters are running at sysclk which is different in PAL/NTSC
	 -- so we count ena ticks and create an adaptive filter_tick signal of approx 250KHz.
	 -- We use use the well oversampled LP output for "unfiltered" audio since the
	 -- 2nd order SD DAC doesn't like sharp transitions.
	 -- There's little point modelling the DC blocking capacitor here since the
	 -- board already has equivalent capacitors on its audio output -- AMR. 

	process(i_sysclk)
	begin
		if rising_edge(i_sysclk) then
			filter_tick<='0';
			if i_sysclk_en='1' then
				if filter_count/="000000" then
					filter_count<=filter_count-1;
				else
					if i_pal='1' then
						filter_count<=to_unsigned(35,6);
					else
						filter_count<=to_unsigned(28,6);
					end if;
					filter_tick<='1';
				end if;
			end if;
		end if;
	end process;

	 -- we use a well oversampled LP output...
	 
	 -- PAL filters	 
   intrinsic_RC_lp: entity work.rc_filter_1o
    generic map (
          highpass_g   => false,
          R_ohms_g     => 1000,      -- 1kOhms   \  LP from output
          C_p_farads_g => 10000,     -- 10 nF    /  with ~16kHz fg
          fclk_hz_g => 250000,
          cwidth_g  => 12,
          dwidthi_g => 6,
          dwidtho_g => 16
    )
    port map (
          clk_i   => i_sysclk,
          clken_i => filter_tick,
          res_i   => i_reset,
          din_i   => vic_audio,
          dout_o  => lp_output
        );

   audio_RC_lp_PAL: entity work.rc_filter_1o
    generic map (
          highpass_g   => false,
          R_ohms_g     => 1000,      -- 1kOhms   \  LP on PCB
          C_p_farads_g => 100000,    -- 100 nF   /  with ~1.6kHz fg
          fclk_hz_g => 250000, -- we use the sysclk / 4 (clk_ena freq)
          cwidth_g  => 14,
          dwidthi_g => 16,
          dwidtho_g => 16
    )
    port map (
          clk_i   => i_sysclk,
          clken_i => filter_tick,
          res_i   => i_reset,
          din_i   => lp_output,
          dout_o  => lp_filtered
        );

  O_AUDIO_FILTERED <= lp_filtered;
  O_AUDIO          <= lp_output;


  via1: entity work.via6522
  port map (
    clock       => i_sysclk,
    rising      => p2_h_rise,
    falling     => p2_h_fall,
    reset       => not reset_l_sampled,

    addr        => c_addr(3 downto 0),
    wen         => c_addr(4) and not io_sel_l(0) and not c_rw_l,
    ren         => c_addr(4) and not io_sel_l(0) and c_rw_l,
    data_in     => v_data(7 downto 0),
    data_out    => via1_dout,

    -- pio --
    port_a_o    => via1_pa_out,
    port_a_i    => via1_pa_in,
    port_b_i    => user_port_in,

    -- handshake pins
    ca1_i       => keybd_restore,

    ca2_o       => cass_motor,
    ca2_i       => cass_motor,

    cb1_i       => user_port_cb1_in,
    cb2_i       => user_port_cb2_in,

    irq         => via1_nmi
  );

  serial_atn_out_l <= via1_pa_out(7);
  via1_pa_in(7) <= serial_atn_in;
  via1_pa_in(6) <= cass_sw;
  via1_pa_in(5) <= light_pen;
  via1_pa_in(4) <= joy(2);
  via1_pa_in(3) <= joy(1);
  via1_pa_in(2) <= joy(0);
  via1_pa_in(1) <= serial_data_in;
  via1_pa_in(0) <= serial_clk_in;

  via2: entity work.via6522
  port map (
    clock       => i_sysclk,
    rising      => p2_h_rise,
    falling     => p2_h_fall,
    reset       => not reset_l_sampled,

    addr        => c_addr(3 downto 0),
    wen         => c_addr(5) and not io_sel_l(0) and not c_rw_l,
    ren         => c_addr(5) and not io_sel_l(0) and c_rw_l,
    data_in     => v_data(7 downto 0),
    data_out    => via2_dout,

		-- pio --
    port_a_o    => keybd_row_out,
    port_a_t    => keybd_row_oe,
    port_a_i    => keybd_row_in,

    port_b_o    => keybd_col_out,
    port_b_t    => keybd_col_oe,
    port_b_i    => keybd_col_in,

    -- handshake pins
    ca1_i       => cass_read,

    ca2_o       => serial_clk_out_l,
    ca2_i       => serial_clk_out_l,
    cb1_i       => serial_srq_in,
    cb2_o       => serial_data_out_l,
    cb2_i       => serial_data_out_l,

    irq         => via2_irq
  );

  cass_write <= keybd_col_out(3);
  keybd_row_out_s <= keybd_row_out or not keybd_row_oe;
  keybd_col_out_s <= keybd_col_out or not keybd_col_oe;

  O_ROW_IN <= not ( keybd_row_out_s );
  keybd_col_in(6 downto 0) <= not( I_COL_OUT(6 downto 0) );
  keybd_col_in(7) <= not( I_COL_OUT(7) ) and joy(3);

  O_COL_IN <= not( keybd_col_out_s );
  keybd_row_in <= not( I_ROW_OUT );
  keybd_restore <= not( I_RESTORE_OUT );

  p_irq_resolve : process(expansion_irq_l, expansion_nmi_l,
                          via2_irq, via1_nmi)
  begin
    c_irq_l <= '1';
    if (expansion_irq_l = '0') or (via2_irq = '1') then
      c_irq_l <= '0';
    end if;

    c_nmi_l <= '1';
    if (expansion_nmi_l = '0') or (via1_nmi = '1') then
      c_nmi_l <= '0';
    end if;
  end process;

  --
  -- decode
  --
  p_io_addr_decode : process(c_addr)
  begin
    io_sel_l <= "1111";
    if (c_addr(15 downto 13) = "100") then -- blk4
      case c_addr(12 downto 10) is
        when "000" => io_sel_l <= "1111";
        when "001" => io_sel_l <= "1111";
        when "010" => io_sel_l <= "1111";
        when "011" => io_sel_l <= "1111";
        when "100" => io_sel_l <= "1110"; -- VIAs
        when "101" => io_sel_l <= "1101"; -- colour RAM
        when "110" => io_sel_l <= "1011";
        when "111" => io_sel_l <= "0111";
        when others => null;
      end case;
    end if;
  end process;

  p_blk_addr_decode : process(c_addr)
  begin
    blk_sel_l <= "11111111";
    case c_addr(15 downto 13) is
      when "000" => blk_sel_l <= "11111110";
      when "001" => blk_sel_l <= "11111101"; -- RAM ext.  ($2000...)
      when "010" => blk_sel_l <= "11111011"; -- RAM ext.  ($4000...)
      when "011" => blk_sel_l <= "11110111"; -- RAM ext.  ($6000...)
      when "100" => blk_sel_l <= "11101111";
      when "101" => blk_sel_l <= "11011111"; -- cartridge ($A000...)
      when "110" => blk_sel_l <= "10111111"; -- basic     ($C000...)
      when "111" => blk_sel_l <= "01111111"; -- kernal    ($E000...)
      when others => null;
    end case;
  end process;

  p_v_mux : process(c_addr, c_dout, c_rw_l, p2_h, vic_addr, v_data_read_mux,
                         blk_sel_l, io_sel_l)
  begin
    -- simplified data source mux
    if (p2_h = '0') then
      v_addr(13 downto 0) <= vic_addr(13 downto 0);
      v_data <= v_data_read_mux(7 downto 0);
      v_rw_l <= '1';
      col_ram_sel_l <= '0';
    else -- cpu
      v_addr(13 downto 0) <= blk_sel_l(4) & c_addr(12 downto 0);
      v_data <= c_dout;
      v_rw_l <= c_rw_l;
      col_ram_sel_l <= io_sel_l(1);
    end if;
  end process;

  p_ram_addr_decode : process(v_addr, blk_sel_l, p2_h)
  begin
    ram_sel_l <= "11111111";
    if ((p2_h = '1') and (blk_sel_l(0) = '0')) or -- cpu
       ((p2_h = '0') and (v_addr(13) = '1')) then
      case v_addr(12 downto 10) is
        when "000" => ram_sel_l <= "11111110"; -- RM        ($0000...)
        when "001" => ram_sel_l <= "11111101"; -- RAM ext.  ($0400...)
        when "010" => ram_sel_l <= "11111011"; -- RAM ext.  ($0800...)
        when "011" => ram_sel_l <= "11110111"; -- RAM ext.  ($0C00...)
        when "100" => ram_sel_l <= "11101111"; -- RAM       ($1000...)
        when "101" => ram_sel_l <= "11011111"; -- RAM       ($1400...)
        when "110" => ram_sel_l <= "10111111"; -- RAM       ($1800...)
        when "111" => ram_sel_l <= "01111111"; -- RAM       ($1C00...)
        when others => null;
      end case;
    end if;
  end process;

  p_vic_din_mux : process(p2_h, col_ram_dout, v_data)
  begin
    if (p2_h = '0') then
      vic_din(11 downto 8) <= col_ram_dout;
    else
      vic_din(11 downto 8) <= v_data(3 downto 0);
    end if;

    vic_din(7 downto 0) <= v_data(7 downto 0);
  end process;

  p_v_read_mux : process(p2_h, col_ram_sel_l, ram_sel_l, vic_oe_l, v_addr,
                         col_ram_dout, ram0_dout, ram45_dout, ram67_dout,
                         vic_dout, char_rom_dout, v_data_read_muxr)
  begin
    -- simplified data read mux
    if (col_ram_sel_l = '0' and p2_h='1') then
      v_data_read_mux <= v_data_read_muxr(7 downto 4) & col_ram_dout;
      v_data_oe_l     <= '0';
    elsif (vic_oe_l = '0') then
      v_data_read_mux <= vic_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(0) = '0') then
      v_data_read_mux <= ram0_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(4) = '0') then
      v_data_read_mux <= ram45_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(5) = '0') then
      v_data_read_mux <= ram45_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(6) = '0') then
      v_data_read_mux <= ram67_dout;
      v_data_oe_l     <= '0';
    elsif (ram_sel_l(7) = '0') then
      v_data_read_mux <= ram67_dout;
      v_data_oe_l     <= '0';
    elsif (v_addr(13 downto 12) = "00") then
      v_data_read_mux <= char_rom_dout;
      v_data_oe_l     <= '0';
    else
      -- take emulated floating bus
      v_data_read_mux <= v_data_read_muxr;
      v_data_oe_l <= '1';
    end if;
  end process;

  -- emulate floating bus with last value kept
  p_v_bus_hold : process
  begin
    wait until rising_edge(I_SYSCLK);
    if (ena_4 = '1') then
      v_data_read_muxr <= v_data_read_mux;
    end if;
  end process;

  p_cpu_read_mux : process(p2_h, c_addr, io_sel_l, ram_sel_l, blk_sel_l,
                           v_data_read_mux, via1_dout, via2_dout, v_data_oe_l,
                           basic_rom_dout, kernal_rom_dout, expansion_din,
                           extmem)
  begin

    if (p2_h = '0') then -- vic is on the bus
      c_din <= "00000000";
    elsif (io_sel_l(0) = '0') and (c_addr(4) = '1') then -- blk4
      c_din <= via1_dout;
    elsif (io_sel_l(0) = '0') and (c_addr(5) = '1') then -- blk5
      c_din <= via2_dout;
    elsif (blk_sel_l(6) = '0' and I_EXTERNAL_ROM = '0') then
      c_din <= basic_rom_dout;
    elsif (blk_sel_l(7) = '0' and I_EXTERNAL_ROM = '0') then
      c_din <= kernal_rom_dout;
    elsif (v_data_oe_l = '0') then
      c_din <= v_data_read_mux;
    elsif (extmem = '1') then
      c_din <= expansion_din; -- everything else --> comes from extensions
    else
      c_din <= x"FF";
    end if;
  end process;
  --
  -- main memory
  --
  rams0 : entity work.ram_conf_1024x8
    generic map (
      START_AI => "000000"   -- 0x0000
    )
    port map (
      CLK     => i_sysclk,
      CLK_EN  => ena_4,
      ENn     => ram_sel_l(0),
      WRn     => v_rw_l,
      ADDR    => v_addr(9 downto 0),
      DIN     => v_data,
      DOUT    => ram0_dout,
      CONF_WR => conf_wr,
      CONF_AI => conf_ai,
      CONF_DI => conf_di
      );
  rams45 : entity work.ram_conf_2048x8
    generic map (
      START_AI => "00010"   -- 0x1000
    )
    port map (
      CLK     => i_sysclk,
      CLK_EN  => ena_4,
      EN1n    => ram_sel_l(4),
      EN2n    => ram_sel_l(5),
      WRn     => v_rw_l,
      ADDR    => v_addr(10 downto 0),
      DIN     => v_data,
      DOUT    => ram45_dout,
      CONF_WR => conf_wr,
      CONF_AI => conf_ai,
      CONF_DI => conf_di
      );
  rams67 : entity work.ram_conf_2048x8
    generic map (
      START_AI => "00011"   -- 0x1800
    )
    port map (
      CLK     => i_sysclk,
      CLK_EN  => ena_4,
      EN1n    => ram_sel_l(6),
      EN2n    => ram_sel_l(7),
      WRn     => v_rw_l,
      ADDR    => v_addr(10 downto 0),
      DIN     => v_data,
      DOUT    => ram67_dout,
      CONF_WR => conf_wr,
      CONF_AI => conf_ai,
      CONF_DI => conf_di
      );
  col_ram : entity work.ram_conf_1024x4
    generic map (
      START_AI => "100101"   -- 0x9400
    )
    port map (
      CLK     => i_sysclk,
      CLK_EN  => ena_4,
      ENn     => col_ram_sel_l,
      WRn     => v_rw_l,
      ADDR    => v_addr(9 downto 0),
      DIN     => v_data(3 downto 0),
      DOUT    => col_ram_dout,
      CONF_WR => conf_wr,
      CONF_AI => conf_ai,
      CONF_DI => conf_di
      );

  --
  -- roms
  --
--  char_rom : entity work.generic_rom
--    generic map (
--      ADDR_WIDTH => 12,
--      START_AI => "1000000000000000",   -- 0x8000
--      FILE_NAME => "characters.901460-03.bin"
--    )
--    port map (
--      CLK     => i_sysclk,
--      ENA     => ena_4,
--      ADDR    => v_addr(11 downto 0),
--      DATA    => char_rom_dout,
--      CONF_WR => conf_wr,
--      CONF_AI => conf_ai,
--      CONF_DI => conf_di
--      );
--
--  basic_rom : entity work.generic_rom
--    generic map (
--      ADDR_WIDTH => 13,
--      START_AI => "1100000000000000",   -- 0xC000
--      FILE_NAME => "basic.901486-01.bin"
--    )
--    port map (
--      CLK     => i_sysclk,
--      ENA     => ena_4,
--      ADDR    => c_addr(12 downto 0),
--      DATA    => basic_rom_dout,
--      CONF_WR => conf_wr,
--      CONF_AI => conf_ai,
--      CONF_DI => conf_di
--      );
--
--  kernal_rom : entity work.generic_rom
--    generic map (
--      ADDR_WIDTH => 13,
--      START_AI => "1110000000000000",   -- 0x8000E000
--      FILE_NAME => "kernal.901486-07.bin"
--    )
--    port map (
--      CLK     => i_sysclk,
--      ENA     => ena_4,
--      ADDR    => c_addr(12 downto 0),
--      DATA    => kernal_rom_dout,
--      CONF_WR => conf_wr,
--      CONF_AI => conf_ai,
--      CONF_DI => conf_di
--      );

  char_wr <= '1' when conf_ai(15 downto 13) = "100" else '0';
  char_rom : entity work.gen_rom
    generic map (
      ADDR_WIDTH => 12,
      --START_AI => "1000000000000000",   -- 0x8000
      INIT_FILE => "../roms/CHARACTERS.HEX"
    )
    port map (
      rdclock   => i_sysclk,
      rdaddress => v_addr(11 downto 0),
      q         => char_rom_dout,
      cs        => '1',
      wrclock   => i_sysclk,
      wren      => conf_wr and char_wr,
      wraddress => conf_ai(11 downto 0),
      data      => conf_di
      );

  basic_wr <= '1' when conf_ai(15 downto 13) = "110" else '0';
  basic_rom : entity work.gen_rom
    generic map (
      ADDR_WIDTH => 13,
      --START_AI => "1100000000000000",   -- 0xC000
      INIT_FILE => "../roms/BASIC.HEX"
    )
    port map (
      rdclock   => i_sysclk,
      rdaddress => c_addr(12 downto 0),
      q         => basic_rom_dout,
      cs        => '1',
      wrclock   => i_sysclk,
      wren      => conf_wr and basic_wr,
      wraddress => conf_ai(12 downto 0),
      data      => conf_di
      );

  process (i_sysclk) begin
    if rising_edge(i_sysclk) then
        if conf_ai = x"ffff" then kernal_pal_written <= '1'; end if;
    end if;
  end process;

  kernal_pal_wr <= '1' when conf_ai(15 downto 13) = "111" and kernal_pal_written = '0' else '0';

  kernal_rom_pal : entity work.gen_rom
    generic map (
      ADDR_WIDTH => 13,
      --START_AI => "1110000000000000",   -- 0xE000
      INIT_FILE => "../roms/KERNAL_PAL.HEX"
    )
    port map (
      rdclock   => i_sysclk,
      rdaddress => c_addr(12 downto 0),
      q         => kernal_pal_dout,
      cs        => '1',
      wrclock   => i_sysclk,
      wren      => conf_wr and kernal_pal_wr,
      wraddress => conf_ai(12 downto 0),
      data      => conf_di
      );

  kernal_ntsc_wr <= '1' when conf_ai(15 downto 13) = "111" and kernal_pal_written = '1' else '0';

  kernal_rom_ntsc : entity work.gen_rom
    generic map (
      ADDR_WIDTH => 13,
      --START_AI => "1110000000000000",   -- 0xE000
      INIT_FILE => "../roms/KERNAL_NTSC.HEX"
    )
    port map (
      rdclock   => i_sysclk,
      rdaddress => c_addr(12 downto 0),
      q         => kernal_ntsc_dout,
      cs        => '1',
      wrclock   => i_sysclk,
      wren      => conf_wr and kernal_ntsc_wr,
      wraddress => conf_ai(12 downto 0),
      data      => conf_di
      );

  kernal_rom_dout <= kernal_pal_dout when i_pal = '1' else kernal_ntsc_dout;

  p_video_output : process
  begin
    wait until rising_edge(i_sysclk);
    if (i_sysclk_en = '1') then
      O_VIDEO_R <= video_r;
      O_VIDEO_G <= video_g;
      O_VIDEO_B <= video_b;
      -- usually sync is always low-active...
      O_HSYNC   <= not hsync;
      O_VSYNC   <= not vsync;
      O_DE      <= de;
    end if;
  end process;

  --
  -- extension memory - connected to external dram controller
  --
  -- at $C000-$FFFF according to I_EXTERNAL_ROM
  -- at $6000(8k),$4000(8k),$2000(8k),$0400(3k) according to I_RAM_EXT
  -- at $A000(8k) according to I_CART_EN
  extmem       <= '1' when (ram_sel_l(1) and ram_sel_l(2) and ram_sel_l(3))='0' and I_RAM_EXT(0)='1' else
                  '1' when blk_sel_l(1)='0' and I_RAM_EXT(1)='1' else
                  '1' when blk_sel_l(2)='0' and I_RAM_EXT(2)='1' else
                  '1' when blk_sel_l(3)='0' and I_RAM_EXT(3)='1' else
                  '1' when blk_sel_l(6)='0' and I_EXTERNAL_ROM = '1' else -- BASIC ROM
                  '1' when blk_sel_l(7)='0' and I_EXTERNAL_ROM = '1' else -- KERNAL ROM
                  '1' when blk_sel_l(5)='0' and I_CART_EN='1' else
                  '0';
  o_extmem_sel <= extmem and p2_h;
  o_extmem_r_wn <= c_rw_l or not blk_sel_l(6) or not blk_sel_l(7) or ( not(blk_sel_l(5)) and I_CART_RO ); -- disable write if we emulate a ROM on $A000
  o_extmem_addr <= c_addr(15 downto 0);
  expansion_din <= i_extmem_data;
  o_extmem_data <= c_dout;

  -- CPU debugging via external lines

  dbg_ext : block is
  signal enable_out : std_logic;
  signal adrsync    : std_logic_vector(15 downto 0);
  signal datsync    : std_logic_vector(7 downto 0);
  signal ctlsync    : std_logic_vector(3 downto 0);
  signal iosync     : std_logic_vector(4 downto 0);
  begin
    ext_dbg_cpu : if ext_dla_cpu_debug=true generate
      debug <= adrsync & datsync & iosync & ctlsync;
      enable_out <= '1' when debugi="111" else                                      -- all
                    '1' when debugi="110" and c_addr(15 downto 12)="1001" else      -- I/O  ($9xxx)
                    '1' when debugi="101" and unsigned(c_addr(15 downto 12))>9 else -- roms (all above $9FFF)
                    '1' when debugi="011" and c_addr(15)='0' else                   -- rams (all below $8000)
                    '0';
      dbgsync : process (I_SYSCLK) is
      begin
        if rising_edge(I_SYSCLK) then
          ctlsync <= c_rw_l & c_nmi_l & c_irq_l & reset_l_sampled;
          iosync <= (not serial_clk_out_l) & (not serial_data_out_l) & atn_i & clk_i & data_i;
          if enable_out='1' then
            if c_rw_l='0' then
              datsync <= c_dout;
            else
              datsync <= c_din;
            end if;
            adrsync <= c_addr(15 downto 0);
          else
            -- disable address in the "selective" modes to identify clearly the access cycle
            if debugi="011" then
              adrsync <= (others => '1'); -- this is the lower range only, so disable with all 1
            elsif debugi/="111" then
              adrsync <= (others => '0'); -- this are higher ranges only, disable with all 0
            end if;
          end if;
        end if;
      end process dbgsync;
    end generate ext_dbg_cpu;
    ext_dbg_cpu_off : if ext_dla_cpu_debug=false generate
      debug <= (others => '0');
    end generate ext_dbg_cpu_off;
  end block dbg_ext;

  -- CPU debugging via ChipScope

  dbg_cs : block is
    --{{{  cs
    component icon
      PORT (
        CONTROL0 : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0)
        );
    end component;

    component ila_1024_63_ext
      PORT (
        CONTROL : INOUT STD_LOGIC_VECTOR(35 DOWNTO 0);
        CLK     : IN STD_LOGIC;
        TRIG0   : IN STD_LOGIC_VECTOR(62 DOWNTO 0)
        );
    end component;

    signal cs_control0   : std_logic_vector(35 downto 0);
    signal cs_clk        : std_logic;
    signal cs_trig       : std_logic_vector(62 downto 0);
    --}}}
  begin
    local_mem_dla : if int_dla_cpu_debug=true generate
      i_icon : icon
        port map (
          CONTROL0 => cs_control0
      );

      i_ila : ila_1024_63_ext
        port map (
          CONTROL => cs_control0,
          CLK     => cs_clk,
          TRIG0   => cs_trig);

      cs_clk       <= I_SYSCLK;

      cs_trig(62 downto 53) <= CONF_AI(15) & CONF_AI(14) & CONF_AI(13) & CONF_AI(12) & CONF_AI(11) & CONF_AI(4) & CONF_AI(3) & CONF_AI(2) & CONF_AI(1) & CONF_AI(0);
      cs_trig(45+7 downto 45) <= CONF_DI;
      cs_trig(44) <= CONF_WR;
      cs_trig(43)  <= ena_4;
      cs_trig(42)  <= cass_write;
      cs_trig(41)  <= extmem;
      cs_trig(40)  <= p2_h;

      cs_trig(32+7 downto 32) <= ram_sel_l;
      cs_trig(24+7 downto 24) <= c_din when c_rw_l='1' else c_dout;
      cs_trig(23)             <= v_data_oe_l;
      cs_trig(22)             <= ena_4;
      cs_trig(5+16 downto 5)  <= c_addr(16 downto 0);

      cs_trig(4)  <= reset_l_sampled;
      cs_trig(3)  <= c_ena;
      cs_trig(2)  <= c_irq_l;
      cs_trig(1)  <= c_nmi_l;
      cs_trig(0)  <= c_rw_l;

    end generate local_mem_dla;
  end block dbg_cs;

end RTL;
