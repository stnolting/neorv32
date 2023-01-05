-- #################################################################################################
-- # << NEORV32 - RISC-V Debug Transport Module (DTM) >>                                           #
-- # ********************************************************************************************* #
-- # Provides a JTAG-compatible TAP to access the DMI register interface.                          #
-- # Compatible to the RISC-V debug specification version 1.0.                                     #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2023, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # https://github.com/stnolting/riscv-debug-dtm                              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;

entity neorv32_debug_dtm is
  generic (
    IDCODE_VERSION : std_ulogic_vector(03 downto 0); -- version
    IDCODE_PARTID  : std_ulogic_vector(15 downto 0); -- part number
    IDCODE_MANID   : std_ulogic_vector(10 downto 0)  -- manufacturer id
  );
  port (
    -- global control --
    clk_i             : in  std_ulogic; -- global clock line
    rstn_i            : in  std_ulogic; -- global reset line, low-active
    -- jtag connection --
    jtag_trst_i       : in  std_ulogic;
    jtag_tck_i        : in  std_ulogic;
    jtag_tdi_i        : in  std_ulogic;
    jtag_tdo_o        : out std_ulogic;
    jtag_tms_i        : in  std_ulogic;
    -- debug module interface (DMI) --
    dmi_req_valid_o   : out std_ulogic;
    dmi_req_ready_i   : in  std_ulogic; -- DMI is allowed to make new requests when set
    dmi_req_address_o : out std_ulogic_vector(05 downto 0);
    dmi_req_data_o    : out std_ulogic_vector(31 downto 0);
    dmi_req_op_o      : out std_ulogic_vector(01 downto 0);
    dmi_rsp_valid_i   : in  std_ulogic; -- response valid when set
    dmi_rsp_ready_o   : out std_ulogic; -- ready to receive response
    dmi_rsp_data_i    : in  std_ulogic_vector(31 downto 0);
    dmi_rsp_op_i      : in  std_ulogic_vector(01 downto 0)
  );
end neorv32_debug_dtm;

architecture neorv32_debug_dtm_rtl of neorv32_debug_dtm is

  -- DMI Configuration (fixed!) --
  constant dmi_idle_c    : std_ulogic_vector(02 downto 0) := "000"; -- no idle cycles required
  constant dmi_version_c : std_ulogic_vector(03 downto 0) := "0001"; -- debug spec. version (0.13 & 1.0)
  constant dmi_abits_c   : std_ulogic_vector(05 downto 0) := "000110"; -- number of DMI address bits (6)

  -- tap JTAG signal synchronizer --
  type tap_sync_t is record
    -- internal --
    trst_ff     : std_ulogic_vector(2 downto 0);
    tck_ff      : std_ulogic_vector(2 downto 0);
    tdi_ff      : std_ulogic_vector(2 downto 0);
    tms_ff      : std_ulogic_vector(2 downto 0);
    -- external --
    trst        : std_ulogic;
    tck_rising  : std_ulogic;
    tck_falling : std_ulogic;
    tdi         : std_ulogic;
    tms         : std_ulogic;
  end record;
  signal tap_sync : tap_sync_t;

  -- tap controller - fsm --
  type tap_ctrl_state_t is (LOGIC_RESET, DR_SCAN, DR_CAPTURE, DR_SHIFT, DR_EXIT1, DR_PAUSE, DR_EXIT2, DR_UPDATE,
                               RUN_IDLE, IR_SCAN, IR_CAPTURE, IR_SHIFT, IR_EXIT1, IR_PAUSE, IR_EXIT2, IR_UPDATE);
  signal tap_ctrl_state : tap_ctrl_state_t;

  -- update trigger --
  type dr_update_trig_t is record
    valid        : std_ulogic;
    is_update    : std_ulogic;
    is_update_ff : std_ulogic;
  end record;
  signal dr_update_trig : dr_update_trig_t;

  -- tap registers --
  type tap_reg_t is record
    ireg             : std_ulogic_vector(04 downto 0);
    bypass           : std_ulogic;
    idcode           : std_ulogic_vector(31 downto 0);
    dtmcs, dtmcs_nxt : std_ulogic_vector(31 downto 0);
    dmi,   dmi_nxt   : std_ulogic_vector((6+32+2)-1 downto 0); -- 6-bit address + 32-bit data + 2-bit operation
  end record;
  signal tap_reg : tap_reg_t;

  -- debug module interface --
  type dmi_ctrl_state_t is (DMI_IDLE, DMI_READ_WAIT, DMI_READ, DMI_READ_BUSY,
                            DMI_WRITE_WAIT, DMI_WRITE, DMI_WRITE_BUSY);
  type dmi_ctrl_t is record
    state        : dmi_ctrl_state_t;
    dmihardreset : std_ulogic;
    dmireset     : std_ulogic;
    rsp          : std_ulogic_vector(01 downto 0); -- sticky response status
    rdata        : std_ulogic_vector(31 downto 0);
    wdata        : std_ulogic_vector(31 downto 0);
    addr         : std_ulogic_vector(05 downto 0);
  end record;
  signal dmi_ctrl : dmi_ctrl_t;

begin

  -- JTAG Input Synchronizer ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tap_synchronizer: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tap_sync.trst_ff <= (others => '0');
      tap_sync.tck_ff  <= (others => '0');
      tap_sync.tdi_ff  <= (others => '0');
      tap_sync.tms_ff  <= (others => '0');
    elsif rising_edge(clk_i) then
      tap_sync.trst_ff <= tap_sync.trst_ff(1 downto 0) & jtag_trst_i;
      tap_sync.tck_ff  <= tap_sync.tck_ff( 1 downto 0) & jtag_tck_i;
      tap_sync.tdi_ff  <= tap_sync.tdi_ff( 1 downto 0) & jtag_tdi_i;
      tap_sync.tms_ff  <= tap_sync.tms_ff( 1 downto 0) & jtag_tms_i;
    end if;
  end process tap_synchronizer;

  -- JTAG reset --
  tap_sync.trst <= '0' when (tap_sync.trst_ff(2 downto 1) = "00") else '1';

  -- JTAG clock edge --
  tap_sync.tck_rising  <= '1' when (tap_sync.tck_ff(2 downto 1) = "01") else '0';
  tap_sync.tck_falling <= '1' when (tap_sync.tck_ff(2 downto 1) = "10") else '0';

  -- JTAG test mode select --
  tap_sync.tms <= tap_sync.tms_ff(2);

  -- JTAG serial data input --
  tap_sync.tdi <= tap_sync.tdi_ff(2);


  -- Tap Control FSM ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  tap_control: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tap_ctrl_state <= LOGIC_RESET;
    elsif rising_edge(clk_i) then
      if (tap_sync.trst = '0') then -- reset
        tap_ctrl_state <= LOGIC_RESET;
      elsif (tap_sync.tck_rising = '1') then -- clock pulse (evaluate TMS on the rising edge of TCK)
        case tap_ctrl_state is -- JTAG state machine
          when LOGIC_RESET => if (tap_sync.tms = '0') then tap_ctrl_state <= RUN_IDLE;   else tap_ctrl_state <= LOGIC_RESET; end if;
          when RUN_IDLE    => if (tap_sync.tms = '0') then tap_ctrl_state <= RUN_IDLE;   else tap_ctrl_state <= DR_SCAN;     end if;
          when DR_SCAN     => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_CAPTURE; else tap_ctrl_state <= IR_SCAN;     end if;
          when DR_CAPTURE  => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_SHIFT;   else tap_ctrl_state <= DR_EXIT1;    end if;
          when DR_SHIFT    => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_SHIFT;   else tap_ctrl_state <= DR_EXIT1;    end if;
          when DR_EXIT1    => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_PAUSE;   else tap_ctrl_state <= DR_UPDATE;   end if;
          when DR_PAUSE    => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_PAUSE;   else tap_ctrl_state <= DR_EXIT2;    end if;
          when DR_EXIT2    => if (tap_sync.tms = '0') then tap_ctrl_state <= DR_SHIFT;   else tap_ctrl_state <= DR_UPDATE;   end if;
          when DR_UPDATE   => if (tap_sync.tms = '0') then tap_ctrl_state <= RUN_IDLE;   else tap_ctrl_state <= DR_SCAN;     end if;
          when IR_SCAN     => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_CAPTURE; else tap_ctrl_state <= LOGIC_RESET; end if;
          when IR_CAPTURE  => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_SHIFT;   else tap_ctrl_state <= IR_EXIT1;    end if;
          when IR_SHIFT    => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_SHIFT;   else tap_ctrl_state <= IR_EXIT1;    end if;
          when IR_EXIT1    => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_PAUSE;   else tap_ctrl_state <= IR_UPDATE;   end if;
          when IR_PAUSE    => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_PAUSE;   else tap_ctrl_state <= IR_EXIT2;    end if;
          when IR_EXIT2    => if (tap_sync.tms = '0') then tap_ctrl_state <= IR_SHIFT;   else tap_ctrl_state <= IR_UPDATE;   end if;
          when IR_UPDATE   => if (tap_sync.tms = '0') then tap_ctrl_state <= RUN_IDLE;   else tap_ctrl_state <= DR_SCAN;     end if;
          when others      => tap_ctrl_state <= LOGIC_RESET;
        end case;
      end if;
    end if;
  end process tap_control;


  -- Tap Register Access --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  reg_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      tap_reg.ireg   <= (others => '0');
      tap_reg.idcode <= (others => '0');
      tap_reg.dtmcs  <= (others => '0');
      tap_reg.dmi    <= (others => '0');
      tap_reg.bypass <= '0';
      jtag_tdo_o     <= '0';
    elsif rising_edge(clk_i) then

      -- serial data input: instruction register --
      if (tap_ctrl_state = LOGIC_RESET) or (tap_ctrl_state = IR_CAPTURE) then -- preload phase
        tap_reg.ireg <= "00001"; -- IDCODE
      elsif (tap_ctrl_state = IR_SHIFT) then -- access phase
        if (tap_sync.tck_rising = '1') then -- [JTAG-SYNC] evaluate TDI on rising edge of TCK
          tap_reg.ireg <= tap_sync.tdi & tap_reg.ireg(tap_reg.ireg'left downto 1);
        end if;
      end if;

      -- serial data input: data register --
      if (tap_ctrl_state = DR_CAPTURE) then -- preload phase
        case tap_reg.ireg is
          when "00001" => tap_reg.idcode <= IDCODE_VERSION & IDCODE_PARTID & IDCODE_MANID & '1'; -- identifier (LSB has to be set)
          when "10000" => tap_reg.dtmcs  <= tap_reg.dtmcs_nxt; -- status register
          when "10001" => tap_reg.dmi    <= tap_reg.dmi_nxt; -- register interface
          when others  => tap_reg.bypass <= '0'; -- pass through
        end case;
      elsif (tap_ctrl_state = DR_SHIFT) then -- access phase
        if (tap_sync.tck_rising = '1') then -- [JTAG-SYNC] evaluate TDI on rising edge of TCK
          case tap_reg.ireg is
            when "00001" => tap_reg.idcode <= tap_sync.tdi & tap_reg.idcode(tap_reg.idcode'left downto 1);
            when "10000" => tap_reg.dtmcs  <= tap_sync.tdi & tap_reg.dtmcs(tap_reg.dtmcs'left downto 1);
            when "10001" => tap_reg.dmi    <= tap_sync.tdi & tap_reg.dmi(tap_reg.dmi'left downto 1);
            when others  => tap_reg.bypass <= tap_sync.tdi;
          end case;
        end if;
      end if;

      -- serial data output --
      if (tap_sync.tck_falling = '1') then -- [JTAG-SYNC] update TDO on falling edge of TCK
        if (tap_ctrl_state = IR_SHIFT) then
          jtag_tdo_o <= tap_reg.ireg(0);
        else
          case tap_reg.ireg is
            when "00001" => jtag_tdo_o <= tap_reg.idcode(0);
            when "10000" => jtag_tdo_o <= tap_reg.dtmcs(0);
            when "10001" => jtag_tdo_o <= tap_reg.dmi(0);
            when others  => jtag_tdo_o <= tap_reg.bypass;
          end case;
        end if;
      end if;

    end if;
  end process reg_access;

  -- DTM Control and Status Register (dtmcs) --
  tap_reg.dtmcs_nxt(31 downto 18) <= (others => '0'); -- unused
  tap_reg.dtmcs_nxt(17)           <= '0'; -- dmihardreset, always reads as zero
  tap_reg.dtmcs_nxt(16)           <= '0'; -- dmireset, always reads as zero
  tap_reg.dtmcs_nxt(15)           <= '0'; -- unused
  tap_reg.dtmcs_nxt(14 downto 12) <= dmi_idle_c; -- minimum number of idle cycles
  tap_reg.dtmcs_nxt(11 downto 10) <= tap_reg.dmi_nxt(1 downto 0); -- dmistat
  tap_reg.dtmcs_nxt(09 downto 04) <= dmi_abits_c; -- number of DMI address bits
  tap_reg.dtmcs_nxt(03 downto 00) <= dmi_version_c; -- version

  -- DMI register read access --
  tap_reg.dmi_nxt(39 downto 34) <= dmi_ctrl.addr; -- address
  tap_reg.dmi_nxt(33 downto 02) <= dmi_ctrl.rdata; -- read data
  tap_reg.dmi_nxt(01 downto 00) <= "11" when (dmi_ctrl.state /= DMI_IDLE) else (dmi_ctrl.rsp); -- status


  -- Debug Module Interface -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_controller: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dmi_ctrl.state        <= DMI_IDLE;
      dmi_ctrl.dmihardreset <= '1';
      dmi_ctrl.dmireset     <= '1';
      dmi_ctrl.rsp          <= "00";
      dmi_ctrl.rdata        <= (others => '0');
      dmi_ctrl.wdata        <= (others => '0');
      dmi_ctrl.addr         <= (others => '0');
    elsif rising_edge(clk_i) then

      -- DMI status and control --
      dmi_ctrl.dmihardreset <= '0'; -- default
      dmi_ctrl.dmireset     <= '0'; -- default
      if (dr_update_trig.valid = '1') and (tap_reg.ireg = "10000") then
        dmi_ctrl.dmireset     <= tap_reg.dtmcs(16);
        dmi_ctrl.dmihardreset <= tap_reg.dtmcs(17);
      end if;

      -- DMI interface arbiter --
      if (dmi_ctrl.dmihardreset = '1') then -- DMI hard reset
        dmi_ctrl.state <= DMI_IDLE;
      else
        case dmi_ctrl.state is

          when DMI_IDLE => -- waiting for new request
            if (dr_update_trig.valid = '1') and (tap_reg.ireg = "10001") then
              dmi_ctrl.addr  <= tap_reg.dmi(39 downto 34);
              dmi_ctrl.wdata <= tap_reg.dmi(33 downto 02);
              if (tap_reg.dmi(1 downto 0) = "01") then -- read
                dmi_ctrl.state <= DMI_READ_WAIT;
              elsif (tap_reg.dmi(1 downto 0) = "10") then -- write
                dmi_ctrl.state <= DMI_WRITE_WAIT;
              end if;
            end if;

          when DMI_READ_WAIT => -- wait for DMI to become ready
            if (dmi_req_ready_i = '1') then
              dmi_ctrl.state <= DMI_READ;
            end if;

          when DMI_READ => -- trigger/start read access
            dmi_ctrl.state <= DMI_READ_BUSY;

          when DMI_READ_BUSY => -- pending read access
            if (dmi_rsp_valid_i = '1') then
              dmi_ctrl.rdata <= dmi_rsp_data_i;
              dmi_ctrl.state <= DMI_IDLE;
            end if;

          when DMI_WRITE_WAIT => -- wait for DMI to become ready
            if (dmi_req_ready_i = '1') then
              dmi_ctrl.state <= DMI_WRITE;
            end if;

          when DMI_WRITE => -- trigger/start write access
            dmi_ctrl.state <= DMI_WRITE_BUSY;

          when DMI_WRITE_BUSY => -- pending write access
            if (dmi_rsp_valid_i = '1') then
              dmi_ctrl.state <= DMI_IDLE;
            end if;

          when others => -- undefined
            dmi_ctrl.state <= DMI_IDLE;

        end case;
      end if;

      -- sticky response flags --
      if (dmi_ctrl.dmireset = '1') or (dmi_ctrl.dmihardreset = '1') then
        dmi_ctrl.rsp <= "00";
      else
        if (dmi_ctrl.state /= DMI_IDLE) and (dr_update_trig.valid = '1') and (tap_reg.ireg = "10001") then -- access attempt while DMI is busy
          dmi_ctrl.rsp <= "11";
        elsif (dmi_ctrl.state = DMI_READ_BUSY) or (dmi_ctrl.state = DMI_WRITE_BUSY) then -- accumulate DMI response
          dmi_ctrl.rsp <= dmi_ctrl.rsp or dmi_rsp_op_i;
        end if;
      end if;

    end if;
  end process dmi_controller;

  -- trigger for UPDATE state --
  tap_update_trigger: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dr_update_trig.is_update_ff <= '0';
    elsif rising_edge(clk_i) then
      dr_update_trig.is_update_ff <= dr_update_trig.is_update;
    end if;
  end process tap_update_trigger;

  dr_update_trig.is_update <= '1' when (tap_ctrl_state = DR_UPDATE) else '0';
  dr_update_trig.valid     <= '1' when (dr_update_trig.is_update = '1') and (dr_update_trig.is_update_ff = '0') else '0';

  -- direct DMI output --
  dmi_req_valid_o   <= '1' when (dmi_ctrl.state = DMI_READ) or (dmi_ctrl.state = DMI_WRITE) else '0';
  dmi_req_op_o      <= tap_reg.dmi(1 downto 0);
  dmi_rsp_ready_o   <= '1' when (dmi_ctrl.state = DMI_READ_BUSY) or (dmi_ctrl.state = DMI_WRITE_BUSY) else '0';
  dmi_req_address_o <= dmi_ctrl.addr;
  dmi_req_data_o    <= dmi_ctrl.wdata;


end neorv32_debug_dtm_rtl;
