-- #################################################################################################
-- # << NEORV32 - RISC-V-Compatible Debug Module (DM) >>                                           #
-- # ********************************************************************************************* #
-- # Compatible to the "Minimal RISC-V External Debug Spec. Version 1.0" using "execution-based"   #
-- # debugging scheme (via the program buffer).                                                    #
-- # ********************************************************************************************* #
-- # Key features:                                                                                 #
-- # * register access commands only                                                               #
-- # * auto-execution commands                                                                     #
-- # * for a single hart only                                                                      #
-- # * 2 general purpose program buffer entries                                                    #
-- # * 1 general purpose data buffer entry                                                         #
-- #                                                                                               #
-- # CPU access:                                                                                   #
-- # * ROM for "park loop" code                                                                    #
-- # * program buffer                                                                              #
-- # * data buffer                                                                                 #
-- # * control and status register                                                                 #
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
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_debug_dm is
  port (
    -- global control --
    clk_i             : in  std_ulogic; -- global clock line
    rstn_i            : in  std_ulogic; -- global reset line, low-active
    -- debug module interface (DMI) --
    dmi_req_valid_i   : in  std_ulogic;
    dmi_req_ready_o   : out std_ulogic; -- DMI is allowed to make new requests when set
    dmi_req_address_i : in  std_ulogic_vector(05 downto 0);
    dmi_req_op_i      : in  std_ulogic_vector(01 downto 0);
    dmi_req_data_i    : in  std_ulogic_vector(31 downto 0);
    dmi_rsp_valid_o   : out std_ulogic; -- response valid when set
    dmi_rsp_ready_i   : in  std_ulogic; -- ready to receive respond
    dmi_rsp_data_o    : out std_ulogic_vector(31 downto 0);
    dmi_rsp_op_o      : out std_ulogic_vector(01 downto 0);
    -- CPU bus access --
    cpu_debug_i       : in  std_ulogic; -- CPU is in debug mode
    cpu_addr_i        : in  std_ulogic_vector(31 downto 0); -- address
    cpu_rden_i        : in  std_ulogic; -- read enable
    cpu_wren_i        : in  std_ulogic; -- write enable
    cpu_ben_i         : in  std_ulogic_vector(03 downto 0); -- byte write enable
    cpu_data_i        : in  std_ulogic_vector(31 downto 0); -- data in
    cpu_data_o        : out std_ulogic_vector(31 downto 0); -- data out
    cpu_ack_o         : out std_ulogic; -- transfer acknowledge
    -- CPU control --
    cpu_ndmrstn_o     : out std_ulogic; -- soc reset
    cpu_halt_req_o    : out std_ulogic  -- request hart to halt (enter debug mode)
  );
end neorv32_debug_dm;

architecture neorv32_debug_dm_rtl of neorv32_debug_dm is

  -- **********************************************************
  -- DMI Access
  -- **********************************************************

  -- DM operations --
  constant dmi_nop_c      : std_ulogic_vector(1 downto 0) := "00"; -- no operation
  constant dmi_read_c     : std_ulogic_vector(1 downto 0) := "01"; -- read data
  constant dmi_write_c    : std_ulogic_vector(1 downto 0) := "10"; -- write data
  constant dmi_reserved_c : std_ulogic_vector(1 downto 0) := "11"; -- reserved

  -- available DMI registers --
  constant addr_data0_c        : std_ulogic_vector(5 downto 0) := "00" & x"4";
  constant addr_dmcontrol_c    : std_ulogic_vector(5 downto 0) := "01" & x"0";
  constant addr_dmstatus_c     : std_ulogic_vector(5 downto 0) := "01" & x"1";
  constant addr_hartinfo_c     : std_ulogic_vector(5 downto 0) := "01" & x"2";
  constant addr_abstractcs_c   : std_ulogic_vector(5 downto 0) := "01" & x"6";
  constant addr_command_c      : std_ulogic_vector(5 downto 0) := "01" & x"7";
  constant addr_abstractauto_c : std_ulogic_vector(5 downto 0) := "01" & x"8";
  constant addr_nextdm_c       : std_ulogic_vector(5 downto 0) := "01" & x"d";
  constant addr_progbuf0_c     : std_ulogic_vector(5 downto 0) := "10" & x"0";
  constant addr_progbuf1_c     : std_ulogic_vector(5 downto 0) := "10" & x"1";
  constant addr_sbcs_c         : std_ulogic_vector(5 downto 0) := "11" & x"8";

  -- RISC-V 32-bit instruction prototypes --
  constant instr_nop_c    : std_ulogic_vector(31 downto 0) := x"00000013"; -- nop
  constant instr_lw_c     : std_ulogic_vector(31 downto 0) := x"00002003"; -- lw zero, 0(zero)
  constant instr_sw_c     : std_ulogic_vector(31 downto 0) := x"00002023"; -- sw zero, 0(zero)
  constant instr_ebreak_c : std_ulogic_vector(31 downto 0) := x"00100073"; -- ebreak

  -- DMI access --
  signal dmi_wren : std_ulogic;
  signal dmi_rden : std_ulogic;

  -- debug module DMI registers / access --
  type progbuf_t is array (0 to 1) of std_ulogic_vector(31 downto 0);
  type dm_reg_t is record
    dmcontrol_ndmreset           : std_ulogic;
    dmcontrol_dmactive           : std_ulogic;
    abstractauto_autoexecdata    : std_ulogic;
    abstractauto_autoexecprogbuf : std_ulogic_vector(01 downto 0);
    progbuf                      : progbuf_t;
    command                      : std_ulogic_vector(31 downto 0);
    --
    halt_req    : std_ulogic;
    resume_req  : std_ulogic;
    reset_ack   : std_ulogic;
    wr_acc_err  : std_ulogic;
    rd_acc_err  : std_ulogic;
    clr_acc_err : std_ulogic;
    autoexec_wr : std_ulogic;
    autoexec_rd : std_ulogic;
  end record;
  signal dm_reg : dm_reg_t;

  -- cpu program buffer --
  type cpu_progbuf_t is array (0 to 4) of std_ulogic_vector(31 downto 0);
  signal cpu_progbuf : cpu_progbuf_t;

  -- **********************************************************
  -- DM Control
  -- **********************************************************

  -- DM configuration --
  constant nscratch_c   : std_ulogic_vector(03 downto 0) := "0001"; -- number of dscratch registers in CPU
  constant datasize_c   : std_ulogic_vector(03 downto 0) := "0001"; -- number of data registers in memory/CSR space
  constant dataaddr_c   : std_ulogic_vector(11 downto 0) := dm_data_base_c(11 downto 0); -- signed base address of data registers in memory/CSR space
  constant dataaccess_c : std_ulogic                     := '1';    -- 1: abstract data is memory-mapped, 0: abstract data is CSR-mapped

  -- debug module controller --
  type dm_ctrl_state_t is (CMD_IDLE, CMD_EXE_CHECK, CMD_EXE_PREPARE, CMD_EXE_TRIGGER, CMD_EXE_BUSY, CMD_EXE_ERROR);
  type dm_ctrl_t is record
    -- fsm --
    state           : dm_ctrl_state_t;
    busy            : std_ulogic;
    ldsw_progbuf    : std_ulogic_vector(31 downto 0);
    pbuf_en         : std_ulogic;
    -- error flags --
    illegal_state   : std_ulogic;
    illegal_cmd     : std_ulogic;
    cmderr          : std_ulogic_vector(02 downto 0);
    -- hart status --
    hart_halted     : std_ulogic;
    hart_resume_req : std_ulogic;
    hart_resume_ack : std_ulogic;
    hart_reset      : std_ulogic;
  end record;
  signal dm_ctrl : dm_ctrl_t;


  -- **********************************************************
  -- CPU Bus Interface
  -- **********************************************************

  -- IO space: module base address --
  constant hi_abb_c : natural := 31; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(dm_size_c); -- low address boundary bit

  -- status and control register - bits --
  -- for write access we only care about the actual BYTE WRITE ACCESSES! --
  constant sreg_halt_ack_c      : natural :=  0; -- -/w: CPU is halted in debug mode and waits in park loop
  constant sreg_resume_req_c    : natural :=  8; -- r/-: DM requests CPU to resume
  constant sreg_resume_ack_c    : natural :=  8; -- -/w: CPU starts resuming
  constant sreg_execute_req_c   : natural := 16; -- r/-: DM requests to execute program buffer
  constant sreg_execute_ack_c   : natural := 16; -- -/w: CPU starts to execute program buffer
  constant sreg_exception_ack_c : natural := 24; -- -/w: CPU has detected an exception

  -- code ROM containing "park loop" --
  -- copied manually from 'sw/ocd-firmware/neorv32_debug_mem_code.vhd' --
  type code_rom_file_t is array (0 to 15) of std_ulogic_vector(31 downto 0);
  constant code_rom_file : code_rom_file_t := (
    00 => x"8c0001a3",
    01 => x"00100073",
    02 => x"7b241073",
    03 => x"8c000023",
    04 => x"8c204403",
    05 => x"00041c63",
    06 => x"8c104403",
    07 => x"fe0408e3",
    08 => x"8c8000a3",
    09 => x"7b202473",
    10 => x"7b200073",
    11 => x"8c000123",
    12 => x"7b202473",
    13 => x"0000100f",
    14 => x"84000067",
    15 => x"00000073"
  );

  -- Debug Core Interface
  type dci_t is record
    halt_ack      : std_ulogic; -- CPU (re-)entered HALT state (single-shot)
    resume_req    : std_ulogic; -- DM wants the CPU to resume when set
    resume_ack    : std_ulogic; -- CPU starts resuming when set (single-shot)
    execute_req   : std_ulogic; -- DM wants CPU to execute program buffer when set
    execute_ack   : std_ulogic; -- CPU starts executing program buffer when set (single-shot)
    exception_ack : std_ulogic; -- CPU has detected an exception (single-shot)
    progbuf       : std_ulogic_vector(255 downto 0); -- program buffer, 4 32-bit entries
    data_we       : std_ulogic; -- write abstract data
    wdata         : std_ulogic_vector(31 downto 0); -- abstract write data
    rdata         : std_ulogic_vector(31 downto 0); -- abstract read data
  end record;
  signal dci : dci_t;

  -- global access control --
  signal acc_en : std_ulogic;
  signal rden   : std_ulogic;
  signal wren   : std_ulogic;
  signal maddr  : std_ulogic_vector(01 downto 0);

  -- data buffer --
  signal data_buf : std_ulogic_vector(31 downto 0);

  -- program buffer access --
  type prog_buf_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal prog_buf : prog_buf_t;

begin

  -- DMI Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_wren <= '1' when (dmi_req_valid_i = '1') and (dmi_req_op_i = dmi_write_c) else '0';
  dmi_rden <= '1' when (dmi_req_valid_i = '1') and (dmi_req_op_i = dmi_read_c)  else '0';


  -- Debug Module Command Controller --------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dm_controller: process(clk_i)
  begin
    if rising_edge(clk_i) then
      if (dm_reg.dmcontrol_dmactive = '0') then -- DM reset / DM disabled
        dm_ctrl.state         <= CMD_IDLE;
        dm_ctrl.ldsw_progbuf  <= instr_sw_c;
        dci.execute_req       <= '0';
        dm_ctrl.pbuf_en       <= '0';
        --
        dm_ctrl.illegal_cmd   <= '0';
        dm_ctrl.illegal_state <= '0';
        dm_ctrl.cmderr        <= "000";
      else -- DM active

        -- defaults --
        dci.execute_req       <= '0';
        dm_ctrl.illegal_cmd   <= '0';
        dm_ctrl.illegal_state <= '0';

        -- command execution engine --
        case dm_ctrl.state is

          when CMD_IDLE => -- wait for new abstract command
          -- ------------------------------------------------------------
            if (dmi_wren = '1') then -- valid DM write access
              if (dmi_req_address_i = addr_command_c) then
                if (dm_ctrl.cmderr = "000") then -- only execute if no error
                  dm_ctrl.state <= CMD_EXE_CHECK;
                end if;
              end if;
            elsif (dm_reg.autoexec_rd = '1') or (dm_reg.autoexec_wr = '1') then -- auto execution trigger
              dm_ctrl.state <= CMD_EXE_CHECK;
            end if;

          when CMD_EXE_CHECK => -- check if command is valid / supported
          -- ------------------------------------------------------------
            if (dm_reg.command(31 downto 24) = x"00") and -- cmdtype: register access
               (dm_reg.command(23) = '0') and -- reserved
               (dm_reg.command(22 downto 20) = "010") and -- aarsize: has to be 32-bit
               (dm_reg.command(19) = '0') and -- aarpostincrement: not supported
               ((dm_reg.command(17) = '0') or (dm_reg.command(15 downto 05) = "00010000000")) then -- regno: only GPRs are supported: 0x1000..0x101f if transfer is set
              if (dm_ctrl.hart_halted = '1') then -- CPU is halted
                dm_ctrl.state <= CMD_EXE_PREPARE;
              else -- error! CPU is still running
                dm_ctrl.illegal_state <= '1';
                dm_ctrl.state         <= CMD_EXE_ERROR;
              end if;
            else -- invalid command
              dm_ctrl.illegal_cmd <= '1';
              dm_ctrl.state       <= CMD_EXE_ERROR;
            end if;

          when CMD_EXE_PREPARE => -- setup program buffer
          -- ------------------------------------------------------------
            if (dm_reg.command(17) = '1') then -- "transfer"
              if (dm_reg.command(16) = '0') then -- "write" = 0 -> read from GPR
                dm_ctrl.ldsw_progbuf <= instr_sw_c;
                dm_ctrl.ldsw_progbuf(31 downto 25) <= dataaddr_c(11 downto 05); -- destination address
                dm_ctrl.ldsw_progbuf(24 downto 20) <= dm_reg.command(4 downto 0); -- "regno" = source register
                dm_ctrl.ldsw_progbuf(11 downto 07) <= dataaddr_c(04 downto 00); -- destination address
              else -- "write" = 1 -> write to GPR
                dm_ctrl.ldsw_progbuf <= instr_lw_c;
                dm_ctrl.ldsw_progbuf(31 downto 20) <= dataaddr_c; -- source address
                dm_ctrl.ldsw_progbuf(11 downto 07) <= dm_reg.command(4 downto 0); -- "regno" = destination register
              end if;
            else
              dm_ctrl.ldsw_progbuf <= instr_nop_c; -- NOP - do nothing
            end if;
            --
            if (dm_reg.command(18) = '1') then -- "postexec" - execute program buffer
              dm_ctrl.pbuf_en <= '1';
            else -- execute all program buffer entries as NOPs
              dm_ctrl.pbuf_en <= '0';
            end if;
            --
            dm_ctrl.state <= CMD_EXE_TRIGGER;

          when CMD_EXE_TRIGGER => -- request CPU to execute command
          -- ------------------------------------------------------------
            dci.execute_req <= '1'; -- request execution
            if (dci.execute_ack = '1') then -- CPU starts execution
              dm_ctrl.state <= CMD_EXE_BUSY;
            end if;

          when CMD_EXE_BUSY => -- wait for CPU to finish
          -- ------------------------------------------------------------
            if (dci.halt_ack = '1') then -- CPU is parked (halted) again -> execution done
              dm_ctrl.state <= CMD_IDLE;
            end if;

          when CMD_EXE_ERROR => -- delay cycle for error to arrive abstracts.cmderr
          -- ------------------------------------------------------------
            dm_ctrl.state <= CMD_IDLE;

          when others => -- undefined
          -- ------------------------------------------------------------
            dm_ctrl.state <= CMD_IDLE;

        end case;

        -- error code --
        -- ------------------------------------------------------------
        if (dm_ctrl.cmderr = "000") then -- ready to set new error
          if (dm_ctrl.illegal_state = '1') then -- cannot execute since hart is not in expected state
            dm_ctrl.cmderr <= "100";
          elsif (dci.exception_ack = '1') then -- exception during execution
            dm_ctrl.cmderr <= "011";
          elsif (dm_ctrl.illegal_cmd = '1') then -- unsupported command
            dm_ctrl.cmderr <= "010";
          elsif (dm_reg.rd_acc_err = '1') or (dm_reg.wr_acc_err = '1') then -- invalid read/write while command is executing
            dm_ctrl.cmderr <= "001";
          end if;
        elsif (dm_reg.clr_acc_err = '1') then -- acknowledge/clear error flags
          dm_ctrl.cmderr <= "000";
        end if;

      end if;
    end if;
  end process dm_controller;

  -- controller busy flag --
  dm_ctrl.busy <= '0' when (dm_ctrl.state = CMD_IDLE) else '1';


  -- Hart Status ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  hart_status: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dm_ctrl.hart_halted     <= '0';
      dm_ctrl.hart_resume_req <= '0';
      dm_ctrl.hart_resume_ack <= '0';
      dm_ctrl.hart_reset      <= '0';
    elsif rising_edge(clk_i) then

      -- HALTED ACK --
      if (dm_reg.dmcontrol_ndmreset = '1') then
        dm_ctrl.hart_halted <= '0';
      elsif (dci.halt_ack = '1') then
        dm_ctrl.hart_halted <= '1';
      elsif (dci.resume_ack = '1') then
        dm_ctrl.hart_halted <= '0';
      end if;

      -- RESUME REQ --
      if (dm_reg.dmcontrol_ndmreset = '1') then
        dm_ctrl.hart_resume_req <= '0';
      elsif (dm_reg.resume_req = '1') then
        dm_ctrl.hart_resume_req <= '1';
      elsif (dci.resume_ack = '1') then
        dm_ctrl.hart_resume_req <= '0';
      end if;

      -- RESUME ACK --
      if (dm_reg.dmcontrol_ndmreset = '1') then
        dm_ctrl.hart_resume_ack <= '0';
      elsif (dci.resume_ack = '1') then
        dm_ctrl.hart_resume_ack <= '1';
      elsif (dm_reg.resume_req = '1') then
        dm_ctrl.hart_resume_ack <= '0';
      end if;

      -- hart has been RESET --
      if (dm_reg.dmcontrol_ndmreset = '1') then -- explicit RESET triggered by DM
        dm_ctrl.hart_reset <= '1';
      elsif (dm_reg.reset_ack = '1') then
        dm_ctrl.hart_reset <= '0';
      end if;

    end if;
  end process hart_status;


  -- Debug Module Interface - Write Access --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dm_reg.dmcontrol_ndmreset <= '0'; -- no system SoC reset
      dm_reg.dmcontrol_dmactive <= '0'; -- DM is in reset state after hardware reset
      --
      dm_reg.abstractauto_autoexecdata    <= '0';
      dm_reg.abstractauto_autoexecprogbuf <= "00";
      --
      dm_reg.command <= (others => '0');
      dm_reg.progbuf <= (others => instr_nop_c);
      --
      dm_reg.halt_req    <= '0';
      dm_reg.resume_req  <= '0';
      dm_reg.reset_ack   <= '0';
      dm_reg.wr_acc_err  <= '0';
      dm_reg.clr_acc_err <= '0';
      dm_reg.autoexec_wr <= '0';
    elsif rising_edge(clk_i) then

      -- default --
      dm_reg.resume_req  <= '0';
      dm_reg.reset_ack   <= '0';
      dm_reg.wr_acc_err  <= '0';
      dm_reg.clr_acc_err <= '0';
      dm_reg.autoexec_wr <= '0';

      -- DMI access --
      if (dmi_wren = '1') then -- valid DMI write request

        -- debug module control --
        if (dmi_req_address_i = addr_dmcontrol_c) then
          dm_reg.halt_req           <= dmi_req_data_i(31); -- haltreq (-/w): write 1 to request halt; has to be cleared again by debugger
          dm_reg.resume_req         <= dmi_req_data_i(30); -- resumereq (-/w1): write 1 to request resume; auto-clears
          dm_reg.reset_ack          <= dmi_req_data_i(28); -- ackhavereset (-/w1): write 1 to ACK reset; auto-clears
          dm_reg.dmcontrol_ndmreset <= dmi_req_data_i(01); -- ndmreset (r/w): soc reset
          dm_reg.dmcontrol_dmactive <= dmi_req_data_i(00); -- dmactive (r/w): DM reset
        end if;

        -- write abstract command --
        if (dmi_req_address_i = addr_command_c) then
          if (dm_ctrl.busy = '0') and (dm_ctrl.cmderr = "000") then -- idle and no errors yet
            dm_reg.command <= dmi_req_data_i;
          end if;
        end if;

        -- write abstract command autoexec --
        if (dmi_req_address_i = addr_abstractauto_c) then
          if (dm_ctrl.busy = '0') then -- idle and no errors yet
            dm_reg.abstractauto_autoexecdata       <= dmi_req_data_i(00);
            dm_reg.abstractauto_autoexecprogbuf(0) <= dmi_req_data_i(16);
            dm_reg.abstractauto_autoexecprogbuf(1) <= dmi_req_data_i(17);
          end if;
        end if;

        -- auto execution trigger --
        if ((dmi_req_address_i = addr_data0_c)    and (dm_reg.abstractauto_autoexecdata = '1')) or
           ((dmi_req_address_i = addr_progbuf0_c) and (dm_reg.abstractauto_autoexecprogbuf(0) = '1')) or
           ((dmi_req_address_i = addr_progbuf1_c) and (dm_reg.abstractauto_autoexecprogbuf(1) = '1')) then
          dm_reg.autoexec_wr <= '1';
        end if;

        -- acknowledge command error --
        if (dmi_req_address_i = addr_abstractcs_c) then
          if (dmi_req_data_i(10 downto 8) = "111") then
            dm_reg.clr_acc_err <= '1';
          end if;
        end if;

        -- write program buffer --
        if (dmi_req_address_i(dmi_req_address_i'left downto 1) = addr_progbuf0_c(dmi_req_address_i'left downto 1)) then
          if (dm_ctrl.busy = '0') then -- idle
            if (dmi_req_address_i(0) = addr_progbuf0_c(0)) then
              dm_reg.progbuf(0) <= dmi_req_data_i;
            else
              dm_reg.progbuf(1) <= dmi_req_data_i;
            end if;
          end if;
        end if;

        -- invalid access while command is executing --
        if (dm_ctrl.busy = '1') then -- busy
          if (dmi_req_address_i = addr_abstractcs_c) or
             (dmi_req_address_i = addr_command_c) or
             (dmi_req_address_i = addr_abstractauto_c) or
             (dmi_req_address_i = addr_data0_c) or
             (dmi_req_address_i = addr_progbuf0_c) or
             (dmi_req_address_i = addr_progbuf1_c) then
            dm_reg.wr_acc_err <= '1';
          end if;
        end if;

      end if;
    end if;
  end process dmi_write_access;


  -- Direct Control -------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  -- write to abstract data register --
  dci.data_we <= '1' when (dmi_wren = '1') and (dmi_req_address_i = addr_data0_c) and (dm_ctrl.busy = '0') else '0';
  dci.wdata   <= dmi_req_data_i;

  -- CPU halt/resume request --
  cpu_halt_req_o <= dm_reg.halt_req and dm_reg.dmcontrol_dmactive; -- single-shot
  dci.resume_req <= dm_ctrl.hart_resume_req; -- active until explicitly cleared

  -- SoC reset --
  cpu_ndmrstn_o <= '0' when (dm_reg.dmcontrol_ndmreset = '1') and (dm_reg.dmcontrol_dmactive = '1') else '1'; -- to processor's reset generator

  -- construct program buffer array for CPU access --
  cpu_progbuf(0) <= dm_ctrl.ldsw_progbuf; -- pseudo program buffer for GPR access
  cpu_progbuf(1) <= instr_nop_c when (dm_ctrl.pbuf_en = '0') else dm_reg.progbuf(0);
  cpu_progbuf(2) <= instr_nop_c when (dm_ctrl.pbuf_en = '0') else dm_reg.progbuf(1);
  cpu_progbuf(3) <= instr_ebreak_c; -- implicit ebreak instruction

  -- DMI status --
  dmi_rsp_op_o    <= "00"; -- operation success
  dmi_req_ready_o <= '1'; -- always ready for new read/write access


  -- Debug Module Interface - Read Access ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      dmi_rsp_valid_o    <= dmi_req_valid_i; -- DMI transfer ack
      dmi_rsp_data_o     <= (others => '0'); -- default
      dm_reg.rd_acc_err  <= '0';
      dm_reg.autoexec_rd <= '0';

      case dmi_req_address_i is

        -- debug module status register --
        when addr_dmstatus_c =>
          dmi_rsp_data_o(31 downto 23) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(22)           <= '1';                       -- impebreak (r/-): there is an implicit ebreak instruction after the visible program buffer
          dmi_rsp_data_o(21 downto 20) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(19)           <= dm_ctrl.hart_reset;        -- allhavereset (r/-): there is only one hart that can be reset
          dmi_rsp_data_o(18)           <= dm_ctrl.hart_reset;        -- anyhavereset (r/-): there is only one hart that can be reset
          dmi_rsp_data_o(17)           <= dm_ctrl.hart_resume_ack;   -- allresumeack (r/-): there is only one hart that can acknowledge resume request
          dmi_rsp_data_o(16)           <= dm_ctrl.hart_resume_ack;   -- anyresumeack (r/-): there is only one hart that can acknowledge resume request
          dmi_rsp_data_o(15)           <= '0';                       -- allnonexistent (r/-): there is only one hart that is always existent
          dmi_rsp_data_o(14)           <= '0';                       -- anynonexistent (r/-): there is only one hart that is always existent
          dmi_rsp_data_o(13)           <= dm_reg.dmcontrol_ndmreset; -- allunavail (r/-): there is only one hart that is unavailable during reset
          dmi_rsp_data_o(12)           <= dm_reg.dmcontrol_ndmreset; -- anyunavail (r/-): there is only one hart that is unavailable during reset
          dmi_rsp_data_o(11)           <= not dm_ctrl.hart_halted;   -- allrunning (r/-): there is only one hart that can be RUNNING or HALTED
          dmi_rsp_data_o(10)           <= not dm_ctrl.hart_halted;   -- anyrunning (r/-): there is only one hart that can be RUNNING or HALTED
          dmi_rsp_data_o(09)           <= dm_ctrl.hart_halted;       -- allhalted (r/-): there is only one hart that can be RUNNING or HALTED
          dmi_rsp_data_o(08)           <= dm_ctrl.hart_halted;       -- anyhalted (r/-): there is only one hart that can be RUNNING or HALTED
          dmi_rsp_data_o(07)           <= '1';                       -- authenticated (r/-): authentication passed since there is no authentication
          dmi_rsp_data_o(06)           <= '0';                       -- authbusy (r/-): always ready since there is no authentication
          dmi_rsp_data_o(05)           <= '0';                       -- hasresethaltreq (r/-): halt-on-reset not implemented
          dmi_rsp_data_o(04)           <= '0';                       -- confstrptrvalid (r/-): no configuration string available
          dmi_rsp_data_o(03 downto 00) <= "0011";                    -- version (r/-): compatible to spec. version 1.0

        -- debug module control --
        when addr_dmcontrol_c =>
          dmi_rsp_data_o(31)           <= '0';                       -- haltreq (-/w): write-only
          dmi_rsp_data_o(30)           <= '0';                       -- resumereq (-/w1): write-only
          dmi_rsp_data_o(29)           <= '0';                       -- hartreset (r/w): not supported
          dmi_rsp_data_o(28)           <= '0';                       -- ackhavereset (-/w1): write-only
          dmi_rsp_data_o(27)           <= '0';                       -- reserved (r/-)
          dmi_rsp_data_o(26)           <= '0';                       -- hasel (r/-) - there is a single currently selected hart
          dmi_rsp_data_o(25 downto 16) <= (others => '0');           -- hartsello (r/-) - there is only one hart
          dmi_rsp_data_o(15 downto 06) <= (others => '0');           -- hartselhi (r/-) - there is only one hart
          dmi_rsp_data_o(05 downto 04) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(03)           <= '0';                       -- setresethaltreq (-/w1): halt-on-reset request - halt-on-reset not implemented
          dmi_rsp_data_o(02)           <= '0';                       -- clrresethaltreq (-/w1): halt-on-reset ack - halt-on-reset not implemented
          dmi_rsp_data_o(01)           <= dm_reg.dmcontrol_ndmreset; -- ndmreset (r/w): soc reset
          dmi_rsp_data_o(00)           <= dm_reg.dmcontrol_dmactive; -- dmactive (r/w): DM reset

        -- hart info --
        when addr_hartinfo_c =>
          dmi_rsp_data_o(31 downto 24) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(23 downto 20) <= nscratch_c;                -- nscratch (r/-): number of dscratch CSRs
          dmi_rsp_data_o(19 downto 17) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(16)           <= dataaccess_c;              -- dataaccess (r/-): 1: data registers are memory-mapped, 0: data reisters are CSR-mapped
          dmi_rsp_data_o(15 downto 12) <= datasize_c;                -- datasize (r/-): number data registers in memory/CSR space
          dmi_rsp_data_o(11 downto 00) <= dataaddr_c;                -- dataaddr (r/-): data registers base address (memory/CSR)

        -- abstract control and status --
        when addr_abstractcs_c =>
          dmi_rsp_data_o(31 downto 24) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(28 downto 24) <= "00010";                   -- progbufsize (r/-): number of words in program buffer = 2
          dmi_rsp_data_o(12)           <= dm_ctrl.busy;              -- busy (r/-): abstract command in progress (1) / idle (0)
          dmi_rsp_data_o(11)           <= '1';                       -- relaxedpriv (r/-): PMP rules are ignored when in debug-mode
          dmi_rsp_data_o(10 downto 08) <= dm_ctrl.cmderr;            -- cmderr (r/w1c): any error during execution?
          dmi_rsp_data_o(07 downto 04) <= (others => '0');           -- reserved (r/-)
          dmi_rsp_data_o(03 downto 00) <= "0001";                    -- datacount (r/-): number of implemented data registers = 1

--      -- abstract command (-/w) --
--      when addr_command_c =>
--        dmi_rsp_data_o <= (others => '0'); -- register is write-only

        -- abstract command autoexec (r/w) --
        when addr_abstractauto_c =>
          dmi_rsp_data_o(00) <= dm_reg.abstractauto_autoexecdata;       -- autoexecdata(0):    read/write access to data0 triggers execution of program buffer
          dmi_rsp_data_o(16) <= dm_reg.abstractauto_autoexecprogbuf(0); -- autoexecprogbuf(0): read/write access to progbuf0 triggers execution of program buffer
          dmi_rsp_data_o(17) <= dm_reg.abstractauto_autoexecprogbuf(1); -- autoexecprogbuf(1): read/write access to progbuf1 triggers execution of program buffer

--      -- next debug module (r/-) --
--      when addr_nextdm_c =>
--        dmi_rsp_data_o <= (others => '0'); -- this is the only DM

        -- abstract data 0 (r/w) --
        when addr_data0_c =>
          dmi_rsp_data_o <= dci.rdata;

--      -- program buffer (-/w) --
--      when addr_progbuf0_c =>
--        dmi_rsp_data_o <= dm_reg.progbuf(0); -- program buffer 0, register is write-only
--      when addr_progbuf1_c =>
--        dmi_rsp_data_o <= dm_reg.progbuf(1); -- program buffer 1, register is write-only

--      -- system bus access control and status (r/-) --
--      when addr_sbcs_c =>
--        dmi_rsp_data_o <= (others => '0'); -- bus access not implemented

        -- not implemented (r/-) --
        when others =>
          dmi_rsp_data_o <= (others => '0');

      end case;

      -- invalid read access while command is executing --
      -- ------------------------------------------------------------
      if (dmi_rden = '1') then -- valid DMI read request
        if (dm_ctrl.busy = '1') then -- busy
          if (dmi_req_address_i = addr_data0_c) or
             (dmi_req_address_i = addr_progbuf0_c) or
             (dmi_req_address_i = addr_progbuf1_c) then
            dm_reg.rd_acc_err <= '1';
          end if;
        end if;
      end if;

      -- auto execution trigger --
      -- ------------------------------------------------------------
      if (dmi_rden = '1') then -- valid DMI read request
        if ((dmi_req_address_i = addr_data0_c)    and (dm_reg.abstractauto_autoexecdata = '1')) or
           ((dmi_req_address_i = addr_progbuf0_c) and (dm_reg.abstractauto_autoexecprogbuf(0) = '1')) or
           ((dmi_req_address_i = addr_progbuf1_c) and (dm_reg.abstractauto_autoexecprogbuf(1) = '1')) then
          dm_reg.autoexec_rd <= '1';
        end if;
      end if;

    end if;
  end process dmi_read_access;


  -- **************************************************************************************************************************
  -- CPU Bus Interface
  -- **************************************************************************************************************************

  -- Access Control ------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  acc_en <= '1' when (cpu_addr_i(hi_abb_c downto lo_abb_c) = dm_base_c(hi_abb_c downto lo_abb_c)) else '0';
  maddr  <= cpu_addr_i(lo_abb_c-1 downto lo_abb_c-2); -- (sub-)module select address
  rden   <= acc_en and cpu_debug_i and cpu_rden_i; -- allow access only when in debug mode
  wren   <= acc_en and cpu_debug_i and cpu_wren_i; -- allow access only when in debug mode


  -- Write Access ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      data_buf          <= (others => '0');
      dci.halt_ack      <= '0';
      dci.resume_ack    <= '0';
      dci.execute_ack   <= '0';
      dci.exception_ack <= '0';
    elsif rising_edge(clk_i) then
      -- data buffer --
      if (dci.data_we = '1') then -- DM write access
        data_buf <= dci.wdata;
      elsif (maddr = "10") and (wren = '1') then -- CPU write access
        data_buf <= cpu_data_i;
      end if;
      -- control and status register CPU write access --
      -- NOTE: we only check the individual BYTE ACCESSES - not the actual write data --
      dci.halt_ack      <= '0'; -- all writable flags auto-clear
      dci.resume_ack    <= '0';
      dci.execute_ack   <= '0';
      dci.exception_ack <= '0';
      if (maddr = "11") and (wren = '1') then
        if (cpu_ben_i(sreg_halt_ack_c/8) = '1') then
          dci.halt_ack <= '1';
        end if;
        if (cpu_ben_i(sreg_resume_ack_c/8) = '1') then
          dci.resume_ack <= '1';
        end if;
        if (cpu_ben_i(sreg_execute_ack_c/8) = '1') then
          dci.execute_ack <= '1';
        end if;
        if (cpu_ben_i(sreg_exception_ack_c/8) = '1') then
          dci.exception_ack <= '1';
        end if;
      end if;
    end if;
  end process write_access;

  -- DM data buffer read access --
  dci.rdata <= data_buf;


  -- Read Access ----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      cpu_ack_o  <= rden or wren;
      cpu_data_o <= (others => '0');
      if (rden = '1') then -- output enable
        case maddr is -- module select
          when "00" => -- code ROM
            cpu_data_o <= code_rom_file(to_integer(unsigned(cpu_addr_i(5 downto 2))));
          when "01" => -- program buffer
            cpu_data_o <= cpu_progbuf(to_integer(unsigned(cpu_addr_i(3 downto 2))));
          when "10" => -- data buffer
            cpu_data_o <= data_buf;
          when others => -- control and status register
            cpu_data_o(sreg_resume_req_c)  <= dci.resume_req;
            cpu_data_o(sreg_execute_req_c) <= dci.execute_req;
        end case;
      end if;
    end if;
  end process read_access;


end neorv32_debug_dm_rtl;
