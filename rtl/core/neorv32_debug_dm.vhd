-- ================================================================================ --
-- NEORV32 OCD - RISC-V-Compatible Debug Module (DM)                                --
-- -------------------------------------------------------------------------------- --
-- Execution-based debugger compatible to the "Minimal RISC-V Debug Specification". --
-- The DM can support up to 4 harts in parallel.                                    --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_debug_dm is
  generic (
    NUM_HARTS     : natural range 1 to 4; -- number of physical CPU cores
    AUTHENTICATOR : boolean -- implement authentication module when true
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic; -- global clock line
    rstn_i     : in  std_ulogic; -- global reset line, low-active
    -- debug module interface (DMI) --
    dmi_req_i  : in  dmi_req_t; -- request
    dmi_rsp_o  : out dmi_rsp_t; -- response
    -- CPU bus access --
    bus_req_i  : in  bus_req_t; -- bus request
    bus_rsp_o  : out bus_rsp_t; -- bus response
    -- CPU control --
    ndmrstn_o  : out std_ulogic; -- soc reset
    halt_req_o : out std_ulogic_vector(NUM_HARTS-1 downto 0) -- request hart to halt (enter debug mode)
  );
end neorv32_debug_dm;

architecture neorv32_debug_dm_rtl of neorv32_debug_dm is

  -- memory map, 128 bytes per device; replicated throughout the entire device address space --
  constant dm_code_base_c : std_ulogic_vector(31 downto 0) := x"ffffff00"; -- code ROM (park loop)
  constant dm_pbuf_base_c : std_ulogic_vector(31 downto 0) := x"ffffff40"; -- program buffer (PBUF)
  constant dm_data_base_c : std_ulogic_vector(31 downto 0) := x"ffffff80"; -- abstract data buffer (DATA)
  constant dm_sreg_base_c : std_ulogic_vector(31 downto 0) := x"ffffffc0"; -- status register (SREG)

  -- ----------------------------------------------------------
  -- Debug Module Interface (DMI)
  -- ----------------------------------------------------------

  -- physical DMI registers --
  constant addr_data0_c        : std_ulogic_vector(6 downto 0) := "0000100";
  constant addr_dmcontrol_c    : std_ulogic_vector(6 downto 0) := "0010000";
  constant addr_dmstatus_c     : std_ulogic_vector(6 downto 0) := "0010001";
  constant addr_hartinfo_c     : std_ulogic_vector(6 downto 0) := "0010010";
  constant addr_abstractcs_c   : std_ulogic_vector(6 downto 0) := "0010110";
  constant addr_command_c      : std_ulogic_vector(6 downto 0) := "0010111";
  constant addr_abstractauto_c : std_ulogic_vector(6 downto 0) := "0011000";
  constant addr_progbuf0_c     : std_ulogic_vector(6 downto 0) := "0100000";
  constant addr_progbuf1_c     : std_ulogic_vector(6 downto 0) := "0100001";
  constant addr_authdata_c     : std_ulogic_vector(6 downto 0) := "0110000";
  constant addr_haltsum0_c     : std_ulogic_vector(6 downto 0) := "1000000";

  -- DMI access --
  signal dmi_wren, dmi_rden : std_ulogic;

  -- ----------------------------------------------------------
  -- DM Control
  -- ----------------------------------------------------------

  -- debug module DMI registers / access --
  type progbuf_t is array (0 to 1) of std_ulogic_vector(31 downto 0);
  type dm_reg_t is record
    ndmreset        : std_ulogic;
    dmactive        : std_ulogic;
    autoexecdata    : std_ulogic;
    autoexecprogbuf : std_ulogic_vector(1 downto 0);
    progbuf         : progbuf_t;
    command         : std_ulogic_vector(31 downto 0);
    halt_req        : std_ulogic;
    req_res         : std_ulogic;
    reset_ack       : std_ulogic;
    hartsel         : std_ulogic_vector(1+1 downto 0); -- plus one bit to detect "unavailable hart"
    set_acc_err     : std_ulogic;
    clr_acc_err     : std_ulogic;
    autoexec        : std_ulogic;
  end record;
  signal dm_reg : dm_reg_t;

  -- currently selected hart --
  signal hartselect : std_ulogic_vector(NUM_HARTS-1 downto 0);
  signal hart_exist : std_ulogic; -- selected hart exists
  signal hart_avail : std_ulogic; -- selected hart is available

  -- CPU program buffer --
  type cpu_progbuf_t is array (0 to 3) of std_ulogic_vector(31 downto 0);
  signal cpu_progbuf : cpu_progbuf_t;

  -- signed base address of data registers in memory/CSR space --
  constant dataaddr_c : std_ulogic_vector(11 downto 0) := dm_data_base_c(11 downto 0);

  -- command execution arbiter --
  type cmd_state_t is (CMD_IDLE, CMD_CHECK, CMD_START, CMD_PENDING);
  signal cmd_state : cmd_state_t;
  signal cmd_err   : std_ulogic_vector(2 downto 0);
  signal cmd_busy  : std_ulogic;
  signal cmd_sw, cmd_lw, cmd_op : std_ulogic_vector(31 downto 0); -- transfer instruction (first progbuf entry)
  constant cmd_nop : std_ulogic_vector(31 downto 0) := x"00000013"; -- RV32 nop
  constant cmd_brk : std_ulogic_vector(31 downto 0) := x"00100073"; -- RV32 ebreak

  -- hart status controller --
  type hart_t is record
    halted, resume_req, resume_ack, reset : std_ulogic_vector(NUM_HARTS-1 downto 0);
  end record;
  signal hart : hart_t;

  -- authenticator interface --
  type auth_t is record
    busy  : std_ulogic; -- authenticator is busy when set
    valid : std_ulogic; -- authentication successful
    re    : std_ulogic; -- data interface read enable
    we    : std_ulogic; -- data interface write enable
    rdata : std_ulogic_vector(31 downto 0); -- read data
  end record;
  signal auth : auth_t;

  -- ----------------------------------------------------------
  -- Debug Core Interface (DCI)
  -- ----------------------------------------------------------

  -- code ROM containing "park loop"; copied manually from 'sw/ocd-firmware/neorv32_application_image.vhd' --
  type code_rom_t is array (0 to 15) of std_ulogic_vector(31 downto 0);
  constant code_rom_c : code_rom_t := (
    x"fc0001a3",
    x"7b241073",
    x"fc000023",
    x"fc204403",
    x"02041063",
    x"fc104403",
    x"fe0408e3",
    x"fc0000a3",
    x"7b202473",
    x"0ff0000f",
    x"0000100f",
    x"7b200073",
    x"fc000123",
    x"7b202473",
    x"f4000067",
    x"00000073"
  );

  -- CPU access helpers --
  signal accen : std_ulogic;
  signal cpu_id_dec : std_ulogic_vector(NUM_HARTS-1 downto 0);

  -- Debug Core Interface --
  type dci_t is record
    ack_hlt  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- CPU (re-)entered HALT state (single-shot)
    req_res  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- DM wants the CPU to resume when set
    ack_res  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- CPU starts resuming when set (single-shot)
    req_exe  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- DM wants CPU to execute program buffer when set
    ack_exe  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- CPU starts executing program buffer when set (single-shot)
    ack_exc  : std_ulogic_vector(NUM_HARTS-1 downto 0); -- CPU has detected an exception (single-shot)
    data_reg : std_ulogic_vector(31 downto 0); -- memory-mapped data exchange register
  end record;
  signal dci : dci_t;

begin

  -- DMI Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_wren <= '1' when (dmi_req_i.op = dmi_req_wr_c) and (auth.valid = '1') and (dm_reg.dmactive = '1') else '0';
  dmi_rden <= '1' when (dmi_req_i.op = dmi_req_rd_c) and (auth.valid = '1') and (dm_reg.dmactive = '1') else '0';


  -- Debug Module Interface - Write Access --------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dm_reg.ndmreset        <= '0';
      dm_reg.dmactive        <= '0';
      dm_reg.autoexecdata    <= '0';
      dm_reg.autoexecprogbuf <= (others => '0');
      dm_reg.progbuf         <= (others => cmd_nop);
      dm_reg.command         <= (others => '0');
      dm_reg.halt_req        <= '0';
      dm_reg.req_res         <= '0';
      dm_reg.reset_ack       <= '0';
      dm_reg.hartsel         <= (others => '0');
      dm_reg.set_acc_err     <= '0';
      dm_reg.clr_acc_err     <= '0';
      dm_reg.autoexec        <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      dm_reg.req_res     <= '0';
      dm_reg.reset_ack   <= '0';
      dm_reg.clr_acc_err <= '0';

      -- ------------------------------------------------------
      -- DMI write access
      -- ------------------------------------------------------
      if (dm_reg.dmactive = '0') then -- DM reset (only reset critical registers)
        dm_reg.ndmreset        <= '0';
        dm_reg.autoexecdata    <= '0';
        dm_reg.autoexecprogbuf <= (others => '0');
        dm_reg.halt_req        <= '0';
        dm_reg.hartsel         <= (others => '0');
      elsif (dmi_wren = '1') then
        case dmi_req_i.addr is

          -- debug module control --
          when addr_dmcontrol_c =>
            dm_reg.halt_req  <= dmi_req_i.data(31);                                          -- haltreq
            dm_reg.req_res   <= dmi_req_i.data(30) and (not dmi_req_i.data(31));             -- resumereq, ignore if halt request
            dm_reg.reset_ack <= dmi_req_i.data(28);                                          -- ackhavereset
            if (cmd_busy = '0') then dm_reg.hartsel <= dmi_req_i.data(18 downto 16); end if; -- hartsello, no update while CMD is executing
            dm_reg.ndmreset  <= dmi_req_i.data(1);                                           -- ndmreset

          -- write abstract command (only when idle and no error yet) --
          when addr_command_c =>
            if (cmd_busy = '0') and (cmd_err = "000") then dm_reg.command <= dmi_req_i.data; end if;

          -- write abstract command autoexec (only when idle) --
          when addr_abstractauto_c =>
            if (cmd_busy = '0') then
              dm_reg.autoexecprogbuf <= dmi_req_i.data(17 downto 16);
              dm_reg.autoexecdata    <= dmi_req_i.data(0);
            end if;

          -- acknowledge command error --
          when addr_abstractcs_c =>
            if (dmi_req_i.data(10 downto 8) = "111") then dm_reg.clr_acc_err <= '1'; end if;

          -- write program buffer 0 (only when idle) --
          when addr_progbuf0_c =>
            if (cmd_busy = '0') then dm_reg.progbuf(0) <= dmi_req_i.data; end if;

          -- write program buffer 1 (only when idle) --
          when addr_progbuf1_c =>
            if (cmd_busy = '0') then dm_reg.progbuf(1) <= dmi_req_i.data; end if;

          -- undefined --
          when others =>
            null;

        end case;
      end if;

      -- dmactive can always be written --
      if (dmi_req_i.op = dmi_req_wr_c) and (dmi_req_i.addr = addr_dmcontrol_c) then
        dm_reg.dmactive <= dmi_req_i.data(0);
      end if;

      -- ------------------------------------------------------
      -- auto-execution triggers
      -- ------------------------------------------------------

      -- trigger execution on data0/progbuf0/progbuf1 access --
      if ((dmi_wren = '1') or (dmi_rden = '1')) and
         (((dmi_req_i.addr = addr_data0_c)    and (dm_reg.autoexecdata = '1')) or
          ((dmi_req_i.addr = addr_progbuf0_c) and (dm_reg.autoexecprogbuf(0) = '1')) or
          ((dmi_req_i.addr = addr_progbuf1_c) and (dm_reg.autoexecprogbuf(1) = '1'))) then
        dm_reg.autoexec <= '1';
      else
        dm_reg.autoexec <= '0';
      end if;

      -- invalid access while command is executing --
      if (cmd_busy = '0') then
        dm_reg.set_acc_err <= '0'; -- reset error trigger when command arbiter is in idle state again
      elsif ((dmi_req_i.addr = addr_abstractcs_c)   and (dmi_wren = '1')) or
            ((dmi_req_i.addr = addr_command_c)      and (dmi_wren = '1')) or
            ((dmi_req_i.addr = addr_abstractauto_c) and (dmi_wren = '1')) or
            ((dmi_req_i.addr = addr_data0_c)        and ((dmi_wren or dmi_rden) = '1')) or
            ((dmi_req_i.addr = addr_progbuf0_c)     and ((dmi_wren or dmi_rden) = '1')) or
            ((dmi_req_i.addr = addr_progbuf1_c)     and ((dmi_wren or dmi_rden) = '1')) then
        dm_reg.set_acc_err <= '1';
      end if;
    end if;
  end process dmi_write_access;

  -- SoC reset --
  ndmrstn_o <= '0' when (dm_reg.ndmreset = '1') and (dm_reg.dmactive = '1') else '1';

  -- hart select --
  hartsel_decode:
  for i in 0 to NUM_HARTS-1 generate
    hartselect(i) <= '1' when (dm_reg.hartsel(2) = '0') and (dm_reg.hartsel(1 downto 0) = std_ulogic_vector(to_unsigned(i, 2))) else '0';
  end generate;
  hart_exist <= '1' when (unsigned(dm_reg.hartsel) < to_unsigned(NUM_HARTS, 3)) else '0'; -- selected hart exists
  hart_avail <= hart_exist and (not dm_reg.ndmreset); -- selected hart is available

  -- CPU halt request --
  request_gen:
  for i in 0 to NUM_HARTS-1 generate
    halt_req_o(i) <= dm_reg.halt_req and hartselect(i) and dm_reg.dmactive;
  end generate;


  -- Debug Module Interface - Read Access ---------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  dmi_read_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      dmi_rsp_o.ack  <= '0';
      dmi_rsp_o.data <= (others => '0');
    elsif rising_edge(clk_i) then
      dmi_rsp_o.ack  <= or_reduce_f(dmi_req_i.op); -- always ACK any request
      dmi_rsp_o.data <= (others => '0');
      case dmi_req_i.addr is

        -- debug module status register --
        when addr_dmstatus_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(24)           <= dm_reg.ndmreset;                                                            -- ndmresetpending
            dmi_rsp_o.data(23)           <= '0';                                                                        -- stickyunavail
            dmi_rsp_o.data(22)           <= '1';                                                                        -- impebreak
            dmi_rsp_o.data(19 downto 18) <= (others => (hart_exist and or_reduce_f(hart.reset      and hartselect)));   -- all/any_havereset
            dmi_rsp_o.data(17 downto 16) <= (others => (hart_avail and or_reduce_f(hart.resume_ack and hartselect)));   -- all/any_resumeack
            dmi_rsp_o.data(15 downto 14) <= (others => (not hart_exist));                                               -- all/any_nonexistent
            dmi_rsp_o.data(13 downto 12) <= (others => (hart_exist and dm_reg.ndmreset));                               -- all/any_unavail
            dmi_rsp_o.data(11 downto 10) <= (others => (hart_avail and (not or_reduce_f(hart.halted and hartselect)))); -- all/any_running
            dmi_rsp_o.data(9 downto 8)   <= (others => (hart_avail and      or_reduce_f(hart.halted and hartselect)));  -- all/any_halted
            dmi_rsp_o.data(5)            <= '0';                                                                        -- hasresethaltreq
            dmi_rsp_o.data(4)            <= '0';                                                                        -- confstrptrvalid
          end if;
          dmi_rsp_o.data(7)          <= auth.valid; -- authenticated
          dmi_rsp_o.data(6)          <= auth.busy;  -- authbusy
          dmi_rsp_o.data(3 downto 0) <= "0011";     -- version

        -- debug module control --
        when addr_dmcontrol_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(31)           <= '0';                        -- haltreq
            dmi_rsp_o.data(30)           <= '0';                        -- resumereq
            dmi_rsp_o.data(29)           <= '0';                        -- hartreset
            dmi_rsp_o.data(28)           <= '0';                        -- ackhavereset
            dmi_rsp_o.data(26)           <= '0';                        -- hasel
            dmi_rsp_o.data(25 downto 16) <= "0000000" & dm_reg.hartsel; -- hartsello
            dmi_rsp_o.data(15 downto 6)  <= "0000000000";               -- hartselhi
            dmi_rsp_o.data(3)            <= '0';                        -- setresethaltreq
            dmi_rsp_o.data(2)            <= '0';                        -- clrresethaltreq
            dmi_rsp_o.data(1)            <= dm_reg.ndmreset;            -- ndmreset
          end if;
          dmi_rsp_o.data(0) <= dm_reg.dmactive; -- dmactive

        -- hart info --
        when addr_hartinfo_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(23 downto 20) <= "0001";                  -- nscratch
            dmi_rsp_o.data(16)           <= '1';                     -- dataaccess
            dmi_rsp_o.data(15 downto 12) <= "0001";                  -- datasize
            dmi_rsp_o.data(11 downto 0)  <= dataaddr_c(11 downto 0); -- dataaddr
          end if;

        -- abstract control and status --
        when addr_abstractcs_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(28 downto 24) <= "00010";  -- progbufsize
            dmi_rsp_o.data(12)           <= cmd_busy; -- busy
            dmi_rsp_o.data(11)           <= '1';      -- relaxedpriv
            dmi_rsp_o.data(10 downto 8)  <= cmd_err;  -- cmderr
            dmi_rsp_o.data(3 downto 0)   <= "0001";   -- datacount
          end if;

        -- abstract command autoexec --
        when addr_abstractauto_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(17 downto 16) <= dm_reg.autoexecprogbuf;
            dmi_rsp_o.data(0)            <= dm_reg.autoexecdata;
          end if;

        -- abstract data 0 --
        when addr_data0_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data <= dci.data_reg;
          end if;

        -- authentication data (can always be read) --
        when addr_authdata_c =>
          dmi_rsp_o.data <= auth.rdata;

        -- halt summary 0 --
        when addr_haltsum0_c =>
          if (dmi_rden = '1') then
            dmi_rsp_o.data(NUM_HARTS-1 downto 0) <= hart.halted(NUM_HARTS-1 downto 0);
          end if;

        -- not implemented or read-only-zero --
        when others =>
          null;

      end case;
    end if;
  end process dmi_read_access;


  -- Hart Status Controller -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  hart_status: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      hart.halted     <= (others => '0');
      hart.resume_req <= (others => '0');
      hart.resume_ack <= (others => '0');
      hart.reset      <= (others => '0');
    elsif rising_edge(clk_i) then
      for i in 0 to NUM_HARTS-1 loop
        -- halted ACK --
        if (dm_reg.ndmreset = '1') or (dm_reg.dmactive = '0') or (dci.ack_res(i) = '1') then
          hart.halted(i) <= '0';
        elsif (dci.ack_hlt(i) = '1') then
          hart.halted(i) <= '1';
        end if;
        -- resume REQ --
        if (dm_reg.ndmreset = '1') or (dm_reg.dmactive = '0') or (dci.ack_res(i) = '1') then
          hart.resume_req(i) <= '0';
        elsif (dm_reg.req_res = '1') and (hartselect(i) = '1') then
          hart.resume_req(i) <= '1';
        end if;
        -- resume ACK --
        if (dm_reg.ndmreset = '1') or (dm_reg.dmactive = '0') or ((dm_reg.req_res = '1') and (hartselect(i) = '1')) then
          hart.resume_ack(i) <= '0';
        elsif (dci.ack_res(i) = '1') then
          hart.resume_ack(i) <= '1';
        end if;
        -- reset ACK --
        if (dm_reg.ndmreset = '1') or (dm_reg.dmactive = '0') then
          hart.reset(i) <= '1';
        elsif (dm_reg.reset_ack = '1') and (hartselect(i) = '1') then
          hart.reset(i) <= '0';
        end if;
      end loop;
    end if;
  end process hart_status;

  -- resume request(s) --
  dci.req_res <= hart.resume_req; -- resume


  -- Command Execution Arbiter --------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  cmd_arbiter: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      cmd_state <= CMD_IDLE;
      cmd_err   <= (others => '0');
    elsif rising_edge(clk_i) then
      if (dm_reg.ndmreset = '1') or (dm_reg.dmactive = '0') then -- hart/DM reset
        cmd_state <= CMD_IDLE;
        cmd_err   <= (others => '0');
      else
        case cmd_state is

          when CMD_IDLE => -- wait for new abstract command
          -- ------------------------------------------------------------
            if (cmd_err = "000") and -- execute only if there is no error yet
               (((dmi_wren = '1') and (dmi_req_i.addr = addr_command_c)) or (dm_reg.autoexec = '1')) then -- manual/auto execution trigger
              cmd_state <= CMD_CHECK;
            end if;
            if (dm_reg.clr_acc_err = '1') then -- clear error flags
              cmd_err <= "000";
            end if;

          when CMD_CHECK => -- check if command is valid / supported
          -- ------------------------------------------------------------
            if (dm_reg.command(31 downto 24) = x"00") and -- cmdtype: register access
               (dm_reg.command(23) = '0') and -- reserved
               (dm_reg.command(19) = '0') and -- aarpostincrement: not supported
               ((dm_reg.command(17) = '0') or -- ignore aarsize and regno if transfer = 0
                ((dm_reg.command(15 downto 5) = "00010000000") and -- regno: only GPRs are supported: 0x1000..0x101f
                 (dm_reg.command(22 downto 20) = "010"))) then -- aarsize == 32-bit
              if (or_reduce_f(hart.halted and hartselect) = '1') then -- selected CPU is halted
                cmd_state <= CMD_START;
              else -- cannot execute since hart is not in expected state
                cmd_err   <= "100";
                cmd_state <= CMD_IDLE;
              end if;
            else -- unsupported command
              cmd_err   <= "010";
              cmd_state <= CMD_IDLE;
            end if;

          when CMD_START => -- trigger execution
          -- ------------------------------------------------------------
            if (or_reduce_f(dci.ack_exe and hartselect) = '1') then -- wait until selected CPU starts executing
              cmd_state <= CMD_PENDING;
            end if;

          when CMD_PENDING => -- wait for CPU to complete execution
          -- ------------------------------------------------------------
            if (cmd_err = "000") then -- cmd_err is sticky - do not alter if already nonzero
              if (or_reduce_f(dci.ack_exc and hartselect) = '1') then -- exception during execution?
                cmd_err <= "011";
              elsif (dm_reg.set_acc_err = '1') then -- invalid read/write while command is executing?
                cmd_err <= "001";
              end if;
            end if;
            -- wait until selected CPU is halted again --
            if (or_reduce_f(dci.ack_hlt and hartselect) = '1') then
              cmd_state <= CMD_IDLE;
            end if;

          when others => -- undefined
          -- ------------------------------------------------------------
            cmd_state <= CMD_IDLE;

        end case;
      end if;
    end if;
  end process cmd_arbiter;

  -- assemble transfer instruction --
  cmd_sw <= dataaddr_c(11 downto 5) & dm_reg.command(4 downto 0) & "00000010" & dataaddr_c(4 downto 0)     & "0100011"; -- store word
  cmd_lw <= dataaddr_c(11 downto 0)                              & "00000010" & dm_reg.command(4 downto 0) & "0000011"; -- load word
  cmd_op <= cmd_nop when (dm_reg.command(17) = '0') else -- no transfer: execute NOP
            cmd_sw  when (dm_reg.command(16) = '0') else cmd_lw; -- write=0: data0 <= GPR (SW); write=1: data0 => GPR (LW)

  -- assemble program buffer --
  cpu_progbuf(0) <= cmd_op; -- pseudo program buffer for GPR<->DM.data0 transfer
  cpu_progbuf(1) <= cmd_nop when (dm_reg.command(18) = '0') else dm_reg.progbuf(0); -- postexec: execute program buffer, else -> NOPs
  cpu_progbuf(2) <= cmd_nop when (dm_reg.command(18) = '0') else dm_reg.progbuf(1);
  cpu_progbuf(3) <= cmd_brk; -- implicit ebreak instruction

  -- controller busy flag --
  cmd_busy <= '0' when (cmd_state = CMD_IDLE) else '1';

  -- request execution --
  dci.req_exe <= hartselect when (cmd_state = CMD_START) else (others => '0');


  -- Bus Access (from CPU) ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o    <= rsp_terminate_c;
      dci.data_reg <= (others => '0');
      dci.ack_hlt  <= (others => '0');
      dci.ack_res  <= (others => '0');
      dci.ack_exe  <= (others => '0');
      dci.ack_exc  <= (others => '0');
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack <= accen;
      bus_rsp_o.err <= '0';
      -- data0 write access --
      if (dmi_wren = '1') and (dmi_req_i.addr = addr_data0_c) and (cmd_busy = '0') then -- DM write access
        dci.data_reg <= dmi_req_i.data;
      elsif (accen = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(7 downto 6) = dm_data_base_c(7 downto 6)) then -- CPU write access
        for i in 0 to 3 loop
          if (bus_req_i.ben(i) = '1') then
            dci.data_reg(8*i+7 downto 8*i) <= bus_req_i.data(8*i+7 downto 8*i);
          end if;
        end loop;
      end if;
      -- CPU status register write access; all flags auto-clear --
      dci.ack_hlt <= (others => '0');
      dci.ack_res <= (others => '0');
      dci.ack_exe <= (others => '0');
      dci.ack_exc <= (others => '0');
      if (accen = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(7 downto 6) = dm_sreg_base_c(7 downto 6)) then
        for i in 0 to NUM_HARTS-1 loop
          if (bus_req_i.ben(0) = '1') then dci.ack_hlt(i) <= cpu_id_dec(i); end if; -- CPU has HALTED
          if (bus_req_i.ben(1) = '1') then dci.ack_res(i) <= cpu_id_dec(i); end if; -- CPU starts RESUMING
          if (bus_req_i.ben(2) = '1') then dci.ack_exe(i) <= cpu_id_dec(i); end if; -- CPU starts to EXECUTE program buffer
          if (bus_req_i.ben(3) = '1') then dci.ack_exc(i) <= cpu_id_dec(i); end if; -- CPU has encountered an EXCEPTION
        end loop;
      end if;
      -- CPU read access --
      bus_rsp_o.data <= (others => '0'); -- default
      if (accen = '1') and (bus_req_i.rw = '0') then
        case bus_req_i.addr(7 downto 6) is -- module select
          when "00" => -- dm_code_base_c: code ROM
            bus_rsp_o.data <= code_rom_c(to_integer(unsigned(bus_req_i.addr(5 downto 2))));
          when "01" => -- dm_pbuf_base_c: program buffer
            bus_rsp_o.data <= cpu_progbuf(to_integer(unsigned(bus_req_i.addr(3 downto 2))));
          when "10" => -- dm_data_base_c: data buffer
            bus_rsp_o.data <= dci.data_reg;
          when others => -- dm_sreg_base_c: status register
            bus_rsp_o.data(1*8) <= or_reduce_f(dci.req_res and cpu_id_dec); -- DM requests CPU to resume
            bus_rsp_o.data(2*8) <= or_reduce_f(dci.req_exe and cpu_id_dec); -- DM requests CPU to execute program buffer
        end case;
      end if;
    end if;
  end process bus_access;

  -- access only when hart is in debug mode --
  accen <= bus_req_i.stb and bus_req_i.meta(2);

  -- CPU ID decoder --
  hart_id_decode_gen:
  for i in 0 to NUM_HARTS-1 generate
    cpu_id_dec(i) <= '1' when (bus_req_i.meta(4 downto 3) = std_ulogic_vector(to_unsigned(i, 2))) or (NUM_HARTS = 1) else '0';
  end generate;


  -- Authentication Module ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  authenticator_enabled:
  if AUTHENTICATOR generate
    neorv32_debug_auth_inst: entity neorv32.neorv32_debug_auth
    port map (
      -- global control --
      clk_i    => clk_i,
      rstn_i   => rstn_i,
      -- register interface --
      we_i     => auth.we,
      re_i     => auth.re,
      wdata_i  => dmi_req_i.data,
      rdata_o  => auth.rdata,
      -- status --
      enable_i => dm_reg.dmactive,
      busy_o   => auth.busy,
      valid_o  => auth.valid
    );
  end generate;

  authenticator_disabled:
  if not AUTHENTICATOR generate
    auth.busy  <= '0';
    auth.valid <= '1'; -- always authenticated
    auth.rdata <= (others => '0');
  end generate;

  -- authenticator access (always accessible) --
  auth.we <= '1' when (dmi_req_i.op = dmi_req_wr_c) and (dmi_req_i.addr = addr_authdata_c) else '0';
  auth.re <= '1' when (dmi_req_i.op = dmi_req_rd_c) and (dmi_req_i.addr = addr_authdata_c) else '0';


end neorv32_debug_dm_rtl;
