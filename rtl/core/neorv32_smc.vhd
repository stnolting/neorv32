-- ================================================================================ --
-- NEORV32 SoC - Serial Memory Controller (SMC)                                     --
-- -------------------------------------------------------------------------------- --
-- Transparently maps up to two external SPI (PS)RAM/flash chips as linear memory   --
-- into the processor address space.                                                --
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

entity neorv32_smc is
  generic (
    BURST_EN   : boolean;                       -- enable burst support
    BURST_SIZE : natural range 4 to 1024;       -- burst size in bytes, has to be a power of 2
    MEM_BASE   : std_ulogic_vector(31 downto 0) -- serial memory base address (256MB-aligned)
  );
  port (
    -- global control --
    clk_i      : in  std_ulogic;                    -- global clock line
    rstn_i     : in  std_ulogic;                    -- global reset line, low-active, async
    -- control and status register interface --
    ctrl_req_i : in  bus_req_t;                     -- bus request
    ctrl_rsp_o : out bus_rsp_t;                     -- bus response
    -- data interface --
    data_req_i : in  bus_req_t;                     -- bus request
    data_rsp_o : out bus_rsp_t;                     -- bus response
    -- serial memory interface --
    smc_ioen_o : out std_ulogic;                    -- SMC pin enable, can be used for IO multiplexing
    smc_sck_o  : out std_ulogic;                    -- clock
    smc_csn_o  : out std_ulogic_vector(1 downto 0); -- bank/chip select, low-active
    smc_sdo_o  : out std_ulogic;                    -- serial data output
    smc_sdi_i  : in  std_ulogic                     -- serial data input
  );
end entity;

architecture neorv32_smc_rtl of neorv32_smc is

  -- byte swap (endianness conversion) --
  function bswap32_f(d : std_ulogic_vector) return std_ulogic_vector is
    variable tmp_v : std_ulogic_vector(31 downto 0);
  begin
    tmp_v := d(7 downto 0) & d(15 downto 8) & d(23 downto 16) & d(31 downto 24);
    return tmp_v;
  end function;

  -- configuration helpers --
  constant log2_fifo_size_c : natural := sel_natural_f(BURST_EN, index_size_f(BURST_SIZE/4), 0);

  -- CSR0 register layout --
  constant csr0_enable_c    : natural :=  0; -- r/w: SMC enable
  constant csr0_ioen_c      : natural :=  1; -- r/w: smc_ioen_o IO pin-enable
--constant csr0_mode_c      : natural :=  2; -- r/w: mode: 0=SPI, 1=QPI [TODO]
  constant csr0_dual_c      : natural :=  3; -- r/w: dual-memory mode enable
  constant csr0_busy_c      : natural :=  4; -- r/-: SMC busy
  --
  constant csr0_msize_lsb_c : natural :=  7; -- r/w: memory size select, 2-bit, LSB
  constant csr0_msize_msb_c : natural :=  8; -- r/w: memory size select, 2-bit, MSB
  constant csr0_cdiv_lsb_c  : natural :=  9; -- r/w: clock divider, 3-bit, LSB
  constant csr0_cdiv_msb_c  : natural := 11; -- r/w: clock divider, 3-bit, MSB
  constant csr0_rwait_lsb_c : natural := 12; -- r/w: read access wait/dummy cycles, 4-bit, LSB
  constant csr0_rwait_msb_c : natural := 15; -- r/w: read access wait/dummy cycles, 4-bit, MSB
  constant csr0_rcmd_lsb_c  : natural := 16; -- r/w: memory read command, 8-bit, LSB
  constant csr0_rcmd_msb_c  : natural := 23; -- r/w: memory read command, 8-bit, MSB
  constant csr0_wcmd_lsb_c  : natural := 24; -- r/w: memory write command, 8bit, LSB
  constant csr0_wcmd_msb_c  : natural := 31; -- r/w: memory write command, 8bit, MSB

  -- configuration registers --
  type csr_t is record
    enable : std_ulogic;                     -- global enable
    ioen   : std_ulogic;                     -- SMC pin-enable
    dual   : std_ulogic;                     -- dual-memory mode enable
    msize  : std_ulogic_vector(1 downto 0);  -- memory size select
    cdiv   : std_ulogic_vector(2 downto 0);  -- clock divider: f_SPI = f_cpu/(2*(cdiv+1))
    rwait  : std_ulogic_vector(3 downto 0);  -- read wait cycles
    cmd_rd : std_ulogic_vector(7 downto 0);  -- read command
    cmd_wr : std_ulogic_vector(7 downto 0);  -- write command
    icmd   : std_ulogic_vector(23 downto 0); -- memory initialization commands (3x8-bit)
  end record;
  signal csr : csr_t; -- register set

  -- data memory access --
  signal acc_req, acc_bank : std_ulogic;
  signal acc_fail : std_ulogic_vector(31 downto 0);

  -- bus access arbiter --
  type state_t is (S_IDLE, S_START, S_BUSY);
  type arb_t is record
    state : state_t;    -- FSM state
    rw    : std_ulogic; -- read/write access
    bcnt  : std_ulogic_vector(8 downto 0);  -- burst counter (number of words = 1024/4=256)
    addr  : std_ulogic_vector(23 downto 0); -- base address buffer
    tsize : std_ulogic_vector(1 downto 0);  -- data transfer quantity select
  end record;
  signal arb, arb_nxt : arb_t; -- FSM
  signal busy : std_ulogic;

  -- write buffer interface --
  signal wbuf_clr, wbuf_we, wbuf_re, wbuf_avail : std_ulogic;
  signal wbuf_rdata : std_ulogic_vector(31 downto 0);

  -- MAC module --
  component neorv32_smc_mac
  port (
    -- global control --
    rstn_i      : in  std_ulogic;
    clk_i       : in  std_ulogic;
    -- configuration --
    cfg_en_i    : in  std_ulogic;
    cfg_icmd_i  : in  std_ulogic_vector(23 downto 0);
    cfg_rcmd_i  : in  std_ulogic_vector(7 downto 0);
    cfg_wcmd_i  : in  std_ulogic_vector(7 downto 0);
    cfg_rwait_i : in  std_ulogic_vector(3 downto 0);
    -- operation control --
    cmd_bank_i  : in  std_ulogic;
    cmd_start_i : in  std_ulogic;
    cmd_rw_i    : in  std_ulogic;
    cmd_size_i  : in  std_ulogic_vector(1 downto 0);
    cmd_addr_i  : in  std_ulogic_vector(23 downto 0);
    cmd_data_i  : in  std_ulogic_vector(31 downto 0);
    cmd_data_o  : out std_ulogic_vector(31 downto 0);
    cmd_busy_o  : out std_ulogic;
    -- memory select --
    mem_csn_o   : out std_ulogic_vector(1 downto 0);
    -- PHY interface --
    phy_start_o : out std_ulogic;
    phy_nbits_o : out std_ulogic_vector(5 downto 0);
    phy_data_o  : out std_ulogic_vector(31 downto 0);
    phy_data_i  : in  std_ulogic_vector(31 downto 0);
    phy_busy_i  : in  std_ulogic
  );
  end component;

  -- MAC interface --
  signal mac_start, mac_busy : std_ulogic;
  signal mac_wdata, mac_rdata : std_ulogic_vector(31 downto 0);

  -- PHY module --
  component neorv32_smc_phy
  port (
    -- global control --
    rstn_i  : in  std_ulogic;
    clk_i   : in  std_ulogic;
    -- operation control --
    en_i    : in  std_ulogic;
    start_i : in  std_ulogic;
    cdiv_i  : in  std_ulogic_vector(2 downto 0);
    nbits_i : in  std_ulogic_vector(5 downto 0);
    busy_o  : out std_ulogic;
    -- RTX data --
    txd_i   : in  std_ulogic_vector(31 downto 0);
    rxd_o   : out std_ulogic_vector(31 downto 0);
    -- memory interface --
    sck_o   : out std_ulogic;
    sdo_o   : out std_ulogic;
    sdi_i   : in  std_ulogic
  );
  end component;

  -- PHY interface --
  signal phy_start, phy_busy : std_ulogic;
  signal phy_nbits : std_ulogic_vector(5 downto 0);
  signal phy_wdata, phy_rdata : std_ulogic_vector(31 downto 0);

begin

  -- Control and Status Registers -----------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ctrl_rsp_o <= rsp_terminate_c;
      csr.enable <= '0';
      csr.ioen   <= '0';
      csr.dual   <= '0';
      csr.msize  <= (others => '0');
      csr.cdiv   <= (others => '0');
      csr.rwait  <= (others => '0');
      csr.cmd_rd <= (others => '0');
      csr.cmd_wr <= (others => '0');
      csr.icmd   <= (others => '0');
    elsif rising_edge(clk_i) then
      -- defaults --
      ctrl_rsp_o.ack  <= ctrl_req_i.stb;
      ctrl_rsp_o.err  <= '0';
      ctrl_rsp_o.data <= (others => '0');
      -- write access --
      if (ctrl_req_i.stb = '1') and (ctrl_req_i.rw = '1') then
        if (ctrl_req_i.addr(2) = '0') then -- CSR0
          csr.enable <= ctrl_req_i.data(csr0_enable_c);
          csr.ioen   <= ctrl_req_i.data(csr0_ioen_c);
          csr.dual   <= ctrl_req_i.data(csr0_dual_c);
          csr.msize  <= ctrl_req_i.data(csr0_msize_msb_c downto csr0_msize_lsb_c);
          csr.cdiv   <= ctrl_req_i.data(csr0_cdiv_msb_c  downto csr0_cdiv_lsb_c);
          csr.rwait  <= ctrl_req_i.data(csr0_rwait_msb_c downto csr0_rwait_lsb_c);
          csr.cmd_rd <= ctrl_req_i.data(csr0_rcmd_msb_c  downto csr0_rcmd_lsb_c);
          csr.cmd_wr <= ctrl_req_i.data(csr0_wcmd_msb_c  downto csr0_wcmd_lsb_c);
        else -- CSR1
          csr.icmd <= ctrl_req_i.data(23 downto 0);
        end if;
      end if;
      -- read access --
      if (ctrl_req_i.stb = '1') and (ctrl_req_i.rw = '0') then
        if (ctrl_req_i.addr(2) = '0') then -- CSR0
          ctrl_rsp_o.data(csr0_enable_c)                            <= csr.enable;
          ctrl_rsp_o.data(csr0_ioen_c)                              <= csr.ioen;
          ctrl_rsp_o.data(csr0_dual_c)                              <= csr.dual;
          ctrl_rsp_o.data(csr0_busy_c)                              <= busy;
          ctrl_rsp_o.data(csr0_msize_msb_c downto csr0_msize_lsb_c) <= csr.msize;
          ctrl_rsp_o.data(csr0_cdiv_msb_c  downto csr0_cdiv_lsb_c)  <= csr.cdiv;
          ctrl_rsp_o.data(csr0_rwait_msb_c downto csr0_rwait_lsb_c) <= csr.rwait;
          ctrl_rsp_o.data(csr0_rcmd_msb_c  downto csr0_rcmd_lsb_c)  <= csr.cmd_rd;
          ctrl_rsp_o.data(csr0_wcmd_msb_c  downto csr0_wcmd_lsb_c)  <= csr.cmd_wr;
        else -- CSR1
          ctrl_rsp_o.data(23 downto 0)  <= csr.icmd;
          ctrl_rsp_o.data(31 downto 28) <= MEM_BASE(31 downto 28);
        end if;
      end if;
    end if;
  end process;

  -- SMC is in charge of interface IOs --
  smc_ioen_o <= csr.ioen;

  -- Memory Address Range Check -------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  mask_gen: process(csr, data_req_i.addr)
    variable tmp_v : std_ulogic_vector(2 downto 0);
  begin
    tmp_v := csr.dual & csr.msize;
    case tmp_v is
      when "000"  => acc_bank <= '0';                 acc_fail <= data_req_i.addr and (not x"f01fffff"); --  1x2 MB
      when "001"  => acc_bank <= '0';                 acc_fail <= data_req_i.addr and (not x"f03fffff"); --  1x4 MB
      when "010"  => acc_bank <= '0';                 acc_fail <= data_req_i.addr and (not x"f07fffff"); --  1x8 MB
      when "011"  => acc_bank <= '0';                 acc_fail <= data_req_i.addr and (not x"f0ffffff"); -- 1x16 MB
      when "100"  => acc_bank <= data_req_i.addr(21); acc_fail <= data_req_i.addr and (not x"f03fffff"); --  2x2 MB
      when "101"  => acc_bank <= data_req_i.addr(22); acc_fail <= data_req_i.addr and (not x"f07fffff"); --  2x4 MB
      when "110"  => acc_bank <= data_req_i.addr(23); acc_fail <= data_req_i.addr and (not x"f0ffffff"); --  2x8 MB
      when "111"  => acc_bank <= data_req_i.addr(24); acc_fail <= data_req_i.addr and (not x"f1ffffff"); -- 2x16 MB
      when others => acc_bank <= '-';                 acc_fail <= (others => '-');
    end case;
  end process;

  -- valid memory request if: SMC enabled, pin access granted and inside memory range --
  acc_req <= '1' when (data_req_i.stb = '1') and (csr.enable = '1') and (csr.ioen = '1') and (acc_fail = x"00000000") else '0';

  -- Write Buffer ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  write_buffer_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => log2_fifo_size_c,
    DWIDTH  => 32,
    OUTGATE => true -- output zero if no write data available (i.e. for read accesses)
  )
  port map (
    -- global control --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => wbuf_clr,
    -- write port --
    wdata_i => data_req_i.data,
    we_i    => wbuf_we,
    free_o  => open, -- no need to check free status: host won't issue more than BURST_SIZE/4 words
    -- read port --
    re_i    => wbuf_re,
    rdata_o => wbuf_rdata,
    avail_o => wbuf_avail
  );

  wbuf_clr <= not csr.enable;
  wbuf_we  <= acc_req and data_req_i.rw;

  -- Access Arbiter Sync --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      arb.state <= S_IDLE;
      arb.rw    <= '0';
      arb.bcnt  <= (others => '0');
      arb.addr  <= (others => '0');
      arb.tsize <= (others => '0');
    elsif rising_edge(clk_i) then
      arb <= arb_nxt;
      if (csr.enable = '0') then -- sync reset only for relevant signals
        arb.state <= S_IDLE;
      end if;
    end if;
  end process;

  -- Access Arbiter Comb --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_comb: process(arb, acc_req, data_req_i, wbuf_avail, wbuf_rdata, mac_busy, mac_rdata)
  begin
    -- defaults --
    arb_nxt    <= arb;
    wbuf_re    <= '0';
    mac_start  <= '0';
    mac_wdata  <= (others => '0');
    data_rsp_o <= rsp_terminate_c;

    -- fsm --
    case arb.state is

      when S_IDLE => -- wait for access request
      -- ------------------------------------------------------------
        if (acc_req = '1') then
          arb_nxt.addr <= data_req_i.addr(23 downto 2) & "00"; -- default = word aligned
          arb_nxt.rw   <= data_req_i.rw;
          if (data_req_i.rw = '0') then -- read
            arb_nxt.tsize <= "11"; -- full word
          else -- write
            case data_req_i.ben is -- data size and address offset
              when "0001" => arb_nxt.tsize <= "00"; arb_nxt.addr(1 downto 0) <= "00"; -- byte 0
              when "0010" => arb_nxt.tsize <= "00"; arb_nxt.addr(1 downto 0) <= "01"; -- byte 1
              when "0100" => arb_nxt.tsize <= "00"; arb_nxt.addr(1 downto 0) <= "10"; -- byte 2
              when "1000" => arb_nxt.tsize <= "00"; arb_nxt.addr(1 downto 0) <= "11"; -- byte 3
              when "0011" => arb_nxt.tsize <= "01"; arb_nxt.addr(1 downto 0) <= "00"; -- low half-word
              when "1100" => arb_nxt.tsize <= "01"; arb_nxt.addr(1 downto 0) <= "10"; -- high half-word
              when others => arb_nxt.tsize <= "11"; arb_nxt.addr(1 downto 0) <= "00"; -- full word
            end case;
          end if;
          if BURST_EN and (data_req_i.burst = '1') then -- burst transfer
            arb_nxt.bcnt <= std_ulogic_vector(to_unsigned(BURST_SIZE/4, 9));
          else -- single access
            arb_nxt.bcnt <= std_ulogic_vector(to_unsigned(1, 9));
          end if;
          arb_nxt.state <= S_START;
        end if;

      when S_START => -- start (another) access operation
      -- ------------------------------------------------------------
        if (wbuf_avail = '1') or (arb.rw = '0') then
          mac_start     <= '1';
          arb_nxt.bcnt  <= std_ulogic_vector(unsigned(arb.bcnt) - 1);
          arb_nxt.state <= S_BUSY;
        end if;

      when S_BUSY => -- access in progress
      -- ------------------------------------------------------------
        mac_wdata       <= bswap32_f(wbuf_rdata); -- little endian: move lowest byte to top (@ADDR+0)
        data_rsp_o.data <= bswap32_f(mac_rdata);
        if (mac_busy = '0') then
          data_rsp_o.ack <= '1'; -- ACK burst element
          arb_nxt.addr   <= std_ulogic_vector(unsigned(arb.addr) + 4); -- next word (relevant only for bursts)
          wbuf_re        <= '1'; -- get next write data (relevant only for write access)
          if (arb.bcnt = "000000000") or (BURST_EN = false) then -- all burst elements done?
            arb_nxt.state <= S_IDLE;
          else -- start next access
            arb_nxt.state <= S_START;
          end if;
        end if;

    end case;
  end process;

  -- access in progress (including MAC initialization) --
  busy <= '0' when (arb.state = S_IDLE) and (mac_busy = '0') else '1';

  -- MAC - Memory Access Controller ---------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_smc_mac_inst: neorv32_smc_mac
  port map (
    -- global control --
    rstn_i      => rstn_i,
    clk_i       => clk_i,
    -- configuration --
    cfg_en_i    => csr.enable,
    cfg_icmd_i  => csr.icmd,
    cfg_rcmd_i  => csr.cmd_rd,
    cfg_wcmd_i  => csr.cmd_wr,
    cfg_rwait_i => csr.rwait,
    -- operation control --
    cmd_bank_i  => acc_bank,
    cmd_start_i => mac_start,
    cmd_rw_i    => arb.rw,
    cmd_size_i  => arb.tsize,
    cmd_addr_i  => arb.addr,
    cmd_data_i  => mac_wdata,
    cmd_data_o  => mac_rdata,
    cmd_busy_o  => mac_busy,
    -- memory select --
    mem_csn_o   => smc_csn_o,
    -- PHY interface --
    phy_start_o => phy_start,
    phy_nbits_o => phy_nbits,
    phy_data_o  => phy_wdata,
    phy_data_i  => phy_rdata,
    phy_busy_i  => phy_busy
  );

  -- PHY - Physical Interface ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  neorv32_smc_phy_inst: neorv32_smc_phy
  port map (
    -- global control --
    rstn_i  => rstn_i,
    clk_i   => clk_i,
    -- operation control --
    en_i    => csr.enable,
    start_i => phy_start,
    cdiv_i  => csr.cdiv,
    nbits_i => phy_nbits,
    busy_o  => phy_busy,
    -- RTX data --
    txd_i   => phy_wdata,
    rxd_o   => phy_rdata,
    -- memory interface --
    sck_o   => smc_sck_o,
    sdo_o   => smc_sdo_o,
    sdi_i   => smc_sdi_i
  );

end architecture;


-- ================================================================================ --
-- NEORV32 SoC - SMC: Memory Access Controller (MAC)                                --
-- -------------------------------------------------------------------------------- --
-- Handles memory access commands and initialization sequence.                      --
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

entity neorv32_smc_mac is
  port (
    -- global control --
    rstn_i     : in  std_ulogic;                      -- global reset line, low-active, async
    clk_i      : in  std_ulogic;                      -- global clock line
    -- configuration --
    cfg_en_i    : in  std_ulogic;                     -- enable, reset when low
    cfg_icmd_i  : in  std_ulogic_vector(23 downto 0); -- initialization commands
    cfg_rcmd_i  : in  std_ulogic_vector(7 downto 0);  -- read command
    cfg_wcmd_i  : in  std_ulogic_vector(7 downto 0);  -- write command
    cfg_rwait_i : in  std_ulogic_vector(3 downto 0);  -- number of read wait cycles (0..15)
    -- operation control --
    cmd_bank_i  : in  std_ulogic;                     -- bank select
    cmd_start_i : in  std_ulogic;                     -- start access
    cmd_rw_i    : in  std_ulogic;                     -- 0=read, 1=write
    cmd_size_i  : in  std_ulogic_vector(1 downto 0);  -- data size select (00=8,01=16,10=24,11=32)
    cmd_addr_i  : in  std_ulogic_vector(23 downto 0); -- access address
    cmd_data_i  : in  std_ulogic_vector(31 downto 0); -- transmit data (MSB-aligned)
    cmd_data_o  : out std_ulogic_vector(31 downto 0); -- receive data (LSB-aligned)
    cmd_busy_o  : out std_ulogic;                     -- operation in progress
    -- memory select --
    mem_csn_o   : out std_ulogic_vector(1 downto 0);  -- bank/chip select, low-active
    -- PHY interface --
    phy_start_o : out std_ulogic;                     -- start transfer
    phy_nbits_o : out std_ulogic_vector(5 downto 0);  -- number of bits to transfer
    phy_data_o  : out std_ulogic_vector(31 downto 0); -- TX data (MSB-aligned)
    phy_data_i  : in  std_ulogic_vector(31 downto 0); -- RX data (LSB-aligned)
    phy_busy_i  : in  std_ulogic                      -- operation in progress
  );
end entity;

architecture neorv32_smc_mac_rtl of neorv32_smc_mac is

  -- access arbiter --
  type state_t is (S_INIT_0, S_INIT_1, S_INIT_2, S_IDLE, S_CMD, S_DUMMY, S_DATA, S_WAIT, S_PAUSE);
  type mac_t is record
    state : state_t;                        -- FSM state
    isel  : std_ulogic_vector(1 downto 0);  -- initialization command select
    csn   : std_ulogic_vector(1 downto 0);  -- bank/chip select, low-active
    rdata : std_ulogic_vector(31 downto 0); -- RX data
  end record;
  signal mac, mac_nxt : mac_t; -- FSM

  -- misc --
  type icmd_t is array (3 downto 0) of std_ulogic_vector(7 downto 0);
  signal icmd : icmd_t; -- initialization commands
  signal bcsn : std_ulogic_vector(1 downto 0); -- bank chip-select
  signal numb : std_ulogic_vector(2 downto 0); -- number of bytes to transfer

begin

  -- Helper Logic ---------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  icmd(0) <= (others => '0');          -- setup command 0; CS disabled
  icmd(1) <= cfg_icmd_i(7 downto 0);   -- setup command 1; CS enabled
  icmd(2) <= cfg_icmd_i(15 downto 8);  -- setup command 2; CS enabled
  icmd(3) <= cfg_icmd_i(23 downto 16); -- setup command 3; CS enabled

  bcsn <= "10" when (cmd_bank_i = '0') else "01"; -- bank/chip select, low-active

  with cmd_size_i select numb <= -- number of bytes to transfer
    "001"           when "00", -- 1 byte
    "010"           when "01", -- 2 bytes
    "011"           when "10", -- 3 bytes
    "100"           when "11", -- 4 bytes
    (others => '-') when others;

  -- Access Arbiter Sync --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_sync: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      mac.state <= S_INIT_0;
      mac.isel  <= (others => '0');
      mac.csn   <= (others => '1');
      mac.rdata <= (others => '0');
    elsif rising_edge(clk_i) then
      mac <= mac_nxt;
      if (cfg_en_i = '0') then -- sync reset only for relevant signals
        mac.state <= S_INIT_0;
        mac.isel  <= (others => '0');
        mac.csn   <= (others => '1');
      end if;
    end if;
  end process;

  -- Access Arbiter Comb --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  arbiter_comb: process(mac, icmd, bcsn, numb, cfg_rcmd_i, cfg_wcmd_i, cfg_rwait_i, cmd_start_i,
                        cmd_rw_i, cmd_addr_i, cmd_data_i, phy_busy_i, phy_data_i)
  begin
    -- defaults --
    mac_nxt     <= mac;
    phy_start_o <= '0';
    phy_nbits_o <= (others => '0');
    phy_data_o  <= (others => '0');

    -- fsm --
    case mac.state is

      when S_INIT_0 => -- wait for PHY to become ready
      -- ------------------------------------------------------------
        mac_nxt.csn <= (others => '1'); -- disable memories
        if (phy_busy_i = '0') then
          mac_nxt.state <= S_INIT_1;
        end if;

      when S_INIT_1 => -- send 8-bit initialization command (SPI-mode)
      -- ------------------------------------------------------------
        if (mac.isel = "00") then -- initial: send dummy clocks without active CS
          mac_nxt.csn <= (others => '1'); -- disable all memories
        else
          mac_nxt.csn <= (others => '0'); -- enable all memories
        end if;
        phy_data_o(31 downto 24) <= icmd(to_integer(unsigned(mac.isel)));
        phy_nbits_o              <= "001000"; -- 8 bits
        phy_start_o              <= '1'; -- trigger transmission
        mac_nxt.state            <= S_INIT_2;

      when S_INIT_2 => -- insert delay and prepare next command
      -- ------------------------------------------------------------
        phy_nbits_o <= "001000"; -- 8 dummy ticks
        if (phy_busy_i = '0') then
          mac_nxt.csn  <= (others => '1'); -- disable memories
          mac_nxt.isel <= std_ulogic_vector(unsigned(mac.isel) + 1);
          if (mac.isel = "11") then -- all commands sent?
            mac_nxt.state <= S_IDLE;
          else -- next initialization command
            phy_start_o   <= '1'; -- trigger transmission of dummy ticks
            mac_nxt.state <= S_INIT_0;
          end if;
        end if;

      when S_IDLE => -- wait for new access
      -- ------------------------------------------------------------
        mac_nxt.csn <= (others => '1'); -- memory disabled
        if (cmd_start_i = '1') then
          mac_nxt.state <= S_CMD;
        end if;

      when S_CMD => -- send command + address (8-bit + 24-bit)
      -- ------------------------------------------------------------
        mac_nxt.csn <= bcsn; -- memory enabled
        phy_nbits_o <= "100000"; -- 8+24=32 bits
        phy_start_o <= '1'; -- trigger transmission
        if (cmd_rw_i = '0') then -- read
          phy_data_o <= cfg_rcmd_i & cmd_addr_i;
          if (cfg_rwait_i = "0000") then -- no wait cycles
            mac_nxt.state <= S_DATA;
          else -- insert dummy cycles
            mac_nxt.state <= S_DUMMY;
          end if;
        else -- write
          phy_data_o    <= cfg_wcmd_i & cmd_addr_i;
          mac_nxt.state <= S_DATA;
        end if;

      when S_DUMMY => -- send dummy bits
      -- ------------------------------------------------------------
        mac_nxt.csn <= bcsn; -- memory enabled
        phy_nbits_o <= "00" & cfg_rwait_i; -- 0..15 cycles
        if (phy_busy_i = '0') then
          phy_start_o   <= '1'; -- trigger transmission
          mac_nxt.state <= S_DATA;
        end if;

      when S_DATA => -- send data
      -- ------------------------------------------------------------
        mac_nxt.csn <= bcsn; -- memory enabled
        phy_data_o  <= cmd_data_i;
        phy_nbits_o <= numb & "000"; -- N*8 bits
        if (phy_busy_i = '0') then
          phy_start_o   <= '1'; -- trigger transmission
          mac_nxt.state <= S_WAIT;
        end if;

      when S_WAIT => -- wait for access to complete
      -- ------------------------------------------------------------
        mac_nxt.csn <= bcsn; -- memory enabled
        phy_nbits_o <= "000010"; -- 2 clock ticks as inter-access delay
        if (phy_busy_i = '0') then
          if (cmd_rw_i = '0') then
            mac_nxt.rdata <= phy_data_i; -- sample RX data
          end if;
          phy_start_o   <= '1'; -- trigger pause transmission
          mac_nxt.state <= S_PAUSE;
        end if;

      when S_PAUSE => -- inter-access delay
      -- ------------------------------------------------------------
        mac_nxt.csn <= (others => '1'); -- memory disabled
        if (phy_busy_i = '0') then
          mac_nxt.state <= S_IDLE;
        end if;

    end case;
  end process;

  -- operation in progress --
  cmd_busy_o <= '0' when (mac.state = S_IDLE) else '1';

  -- bank/chip select --
  mem_csn_o <= mac.csn;

  -- RX data --
  cmd_data_o <= mac.rdata;

end architecture;


-- ================================================================================ --
-- NEORV32 SoC - SMC: Physical Interface (PHY)                                      --
-- -------------------------------------------------------------------------------- --
-- Simple serializer/deserializer.                                                  --
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

entity neorv32_smc_phy is
  port (
    -- global control --
    rstn_i  : in  std_ulogic;                     -- reset, async, low-active
    clk_i   : in  std_ulogic;                     -- clock
    -- operation control --
    en_i    : in  std_ulogic;                     -- module enable (reset when low)
    start_i : in  std_ulogic;                     -- start transfer (single-shot)
    cdiv_i  : in  std_ulogic_vector(2 downto 0);  -- clock divider
    nbits_i : in  std_ulogic_vector(5 downto 0);  -- number of bits to transfer (min 1)
    busy_o  : out std_ulogic;                     -- operation in progress
    -- RTX data --
    txd_i   : in  std_ulogic_vector(31 downto 0); -- TX data (MSB-aligned)
    rxd_o   : out std_ulogic_vector(31 downto 0); -- RX data (LSB-aligned)
    -- memory interface --
    sck_o   : out std_ulogic;                     -- clock
    sdo_o   : out std_ulogic;                     -- output data
    sdi_i   : in  std_ulogic                      -- input data
  );
end entity;

architecture neorv32_smc_phy_rtl of neorv32_smc_phy is

  -- serial engine --
  type state_t is (S_IDLE, S_RTX_0, S_RTX_1);
  signal state : state_t;                        -- FSM state
  signal sreg  : std_ulogic_vector(31 downto 0); -- input/output shift register
  signal cdiv  : std_ulogic_vector(2 downto 0);  -- clock divider
  signal bcnt  : std_ulogic_vector(5 downto 0);  -- bit counter
  signal sck   : std_ulogic;                     -- serial clock
  signal sdi   : std_ulogic;                     -- input sample register

begin

  -- Serial Interface Engine ----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  serial_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      state <= S_IDLE;
      sreg  <= (others => '0');
      cdiv  <= (others => '0');
      bcnt  <= (others => '0');
      sck   <= '0';
      sdi   <= '0';
    elsif rising_edge(clk_i) then
      sdi <= sdi_i; -- input synchronizer
      case state is

        when S_IDLE => -- wait for request and sample configuration
        -- ------------------------------------------------------------
          sck  <= '0'; -- clock mode 0: low when idle
          cdiv <= cdiv_i; -- reload clock counter
          bcnt <= nbits_i;
          sreg <= txd_i;
          if (en_i = '1') and (start_i = '1') then
            state <= S_RTX_0;
          end if;

        when S_RTX_0 => -- low clock phase (1/2 T_clk)
        -- ------------------------------------------------------------
          if (en_i = '0') then -- shutdown
            state <= S_IDLE;
          elsif (cdiv = "000") then -- end of phase
            sck   <= '1'; -- rising edge
            cdiv  <= cdiv_i; -- reload clock counter
            bcnt  <= std_ulogic_vector(unsigned(bcnt) - 1);
            state <= S_RTX_1;
          else
            cdiv <= std_ulogic_vector(unsigned(cdiv) - 1);
          end if;

        when S_RTX_1 => -- high clock phase (1/2 T_clk)
        -- ------------------------------------------------------------
          if (en_i = '0') then -- shutdown
            state <= S_IDLE;
          elsif (cdiv = "000") then -- end of phase
            sck  <= '0'; -- falling edge
            cdiv <= cdiv_i; -- reload clock counter
            sreg <= sreg(30 downto 0) & sdi; -- set & sample at falling edge
            if (bcnt = "000000") then
              state <= S_IDLE;
            else
              state <= S_RTX_0;
            end if;
          else
            cdiv <= std_ulogic_vector(unsigned(cdiv) - 1);
          end if;

      end case;
    end if;
  end process;

  -- transfer in progress --
  busy_o <= '0' when (state = S_IDLE) else '1';

  -- RX data --
  rxd_o <= sreg;

  -- serial output --
  sck_o <= sck;
  sdo_o <= sreg(31);

end architecture;
