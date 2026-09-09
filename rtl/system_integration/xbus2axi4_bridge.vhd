-- ================================================================================ --
-- NEORV32 SoC - Size-Optimized XBUS to AXI4-Compatible Bridge                      --
-- -------------------------------------------------------------------------------- --
-- Supported transfers: Single Transfers and Incrementing Address Bursts.           --
-- [TODO] Exclusive accesses do not check for EXOKAY.                               --
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

entity xbus2axi4_bridge is
  generic (
    BURST_EN  : boolean; -- enable burst transfers
    BURST_LEN : natural range 4 to 1024 -- bytes per burst, has to be a multiple of 4
  );
  port (
    -- Global control --
    clk           : in  std_logic;
    resetn        : in  std_logic;
    -- XBUS device interface --
    xbus_adr_i    : in  std_ulogic_vector(31 downto 0);
    xbus_dat_i    : in  std_ulogic_vector(31 downto 0);
    xbus_cti_i    : in  std_ulogic_vector(2 downto 0);
    xbus_tag_i    : in  std_ulogic_vector(2 downto 0);
    xbus_we_i     : in  std_ulogic;
    xbus_sel_i    : in  std_ulogic_vector(3 downto 0);
    xbus_stb_i    : in  std_ulogic;
    xbus_cyc_i    : in  std_ulogic;
    xbus_ack_o    : out std_ulogic;
    xbus_err_o    : out std_ulogic;
    xbus_dat_o    : out std_ulogic_vector(31 downto 0);
    -- AXI4 host write address channel --
    m_axi_awaddr  : out std_logic_vector(31 downto 0);
    m_axi_awlen   : out std_logic_vector(7 downto 0);
    m_axi_awsize  : out std_logic_vector(2 downto 0);
    m_axi_awburst : out std_logic_vector(1 downto 0);
    m_axi_awcache : out std_logic_vector(3 downto 0);
    m_axi_awprot  : out std_logic_vector(2 downto 0);
    m_axi_awvalid : out std_logic;
    m_axi_awready : in  std_logic;
    -- AXI4 host write data channel --
    m_axi_wdata   : out std_logic_vector(31 downto 0);
    m_axi_wstrb   : out std_logic_vector(3 downto 0);
    m_axi_wlast   : out std_logic;
    m_axi_wvalid  : out std_logic;
    m_axi_wready  : in  std_logic;
    -- AXI4 host read address channel --
    m_axi_araddr  : out std_logic_vector(31 downto 0);
    m_axi_arlen   : out std_logic_vector(7 downto 0);
    m_axi_arsize  : out std_logic_vector(2 downto 0);
    m_axi_arburst : out std_logic_vector(1 downto 0);
    m_axi_arcache : out std_logic_vector(3 downto 0);
    m_axi_arprot  : out std_logic_vector(2 downto 0);
    m_axi_arvalid : out std_logic;
    m_axi_arready : in  std_logic;
    -- AXI4 host read data channel --
    m_axi_rdata   : in  std_logic_vector(31 downto 0);
    m_axi_rresp   : in  std_logic_vector(1 downto 0);
    m_axi_rlast   : in  std_logic;
    m_axi_rvalid  : in  std_logic;
    m_axi_rready  : out std_logic;
    -- AXI4 host write response channel --
    m_axi_bresp   : in  std_logic_vector(1 downto 0);
    m_axi_bvalid  : in  std_logic;
    m_axi_bready  : out std_logic
  );
end entity;

architecture xbus2axi4_bridge_rtl of xbus2axi4_bridge is

  -- determine FIFO address width --
  function fifo_awidth_f(e : boolean; n : natural) return natural is
  begin
    if e then
      for i in 0 to 10 loop
        if (2**i >= n) then
          return i;
        end if;
      end loop;
      return 10; -- fallback
    else
      return 0; -- no bursts, use a single-entry FIFO
    end if;
  end function;

  -- auto-configuration --
  constant fifo_awidth_c : natural := fifo_awidth_f(BURST_EN, BURST_LEN/4);
  constant blen_c : std_ulogic_vector(7 downto 0) := std_ulogic_vector(to_unsigned((BURST_LEN/4)-1, 8));

  -- FIFO interface --
  type fifo_t is record
    we,    re    : std_ulogic;
    wdata, rdata : std_ulogic_vector(32 downto 0);
    avail, clr   : std_ulogic;
  end record;
  signal fifo : fifo_t;

  -- arbitration --
  type state_t is (S_IDLE, S_SINGLE_READ, S_SINGLE_WRITE, S_BURST_READ, S_BURST_WRITE, S_BURST_END);
  signal state : state_t;
  signal busy, burst, rw, w_ack : std_ulogic;
  signal arvalid, awvalid, xbus_rd_ack, xbus_rd_err, xbus_wr_ack, xbus_wr_err : std_ulogic;
  signal address : std_ulogic_vector(31 downto 0);

begin

  -- Address Channels -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  address_arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      arvalid <= '0';
      awvalid <= '0';
      address <= (others => '0');
    elsif rising_edge(clk) then
      if (busy = '0') then -- idle
        arvalid <= '0';
        awvalid <= '0';
        if (xbus_cyc_i = '1') and (xbus_stb_i = '1') then -- mew access request
          arvalid <= not xbus_we_i;
          awvalid <= xbus_we_i;
          address <= xbus_adr_i;
        end if;
      else
        arvalid <= arvalid and std_ulogic(not m_axi_arready);
        awvalid <= awvalid and std_ulogic(not m_axi_awready);
      end if;
    end if;
  end process;

  -- AXI read address channel --
  m_axi_araddr  <= std_logic_vector(address);
  m_axi_arlen   <= std_logic_vector(blen_c) when BURST_EN and (burst = '1') else (others => '0'); -- burst length
  m_axi_arsize  <= "010"; -- 4 bytes per beat
  m_axi_arburst <= "01"; -- incrementing bursts only
  m_axi_arcache <= "0011"; -- recommended by Vivado
  m_axi_arprot  <= std_logic_vector(xbus_tag_i);
  m_axi_arvalid <= std_logic(arvalid);

  -- AXI write address channel --
  m_axi_awaddr  <= std_logic_vector(address);
  m_axi_awlen   <= std_logic_vector(blen_c) when BURST_EN and (burst = '1') else (others => '0'); -- burst length
  m_axi_awsize  <= "010"; -- 4 bytes per beat
  m_axi_awburst <= "01"; -- incrementing bursts only
  m_axi_awcache <= "0011"; -- recommended by Vivado
  m_axi_awprot  <= std_logic_vector(xbus_tag_i);
  m_axi_awvalid <= std_logic(awvalid);


  -- Write Data Channel ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  data_buffer_inst: entity neorv32.neorv32_prim_fifo
  generic map (
    AWIDTH  => fifo_awidth_c,
    DWIDTH  => 33,
    OUTGATE => false,
    ASYNCRD => true
  )
  port map (
    -- global control --
    clk_i   => clk,
    rstn_i  => resetn,
    clear_i => fifo.clr,
    -- write port --
    wdata_i => fifo.wdata,
    we_i    => fifo.we,
    free_o  => open,
    -- read port --
    re_i    => fifo.re,
    rdata_o => fifo.rdata,
    avail_o => fifo.avail
  );

  fifo.clr <= not xbus_cyc_i;
  fifo.we  <= xbus_cyc_i and xbus_stb_i and xbus_we_i;
  fifo.re  <= fifo.avail and std_ulogic(m_axi_wready);

  fifo.wdata(32) <= '1' when (xbus_cti_i = "000") or (xbus_cti_i = "001") else '0'; -- last/only word of transfer
  fifo.wdata(31 downto 0) <= xbus_dat_i;

  -- AXI write data channel --
  m_axi_wdata  <= std_logic_vector(fifo.rdata(31 downto 0));
  m_axi_wstrb  <= std_logic_vector(xbus_sel_i);
  m_axi_wlast  <= std_logic(fifo.rdata(32)); -- last word of transfer
  m_axi_wvalid <= std_logic(fifo.avail);


  -- Transfer Arbiter -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  access_arbiter: process(resetn, clk)
  begin
    if (resetn = '0') then
      state <= S_IDLE;
      w_ack <= '0';
    elsif rising_edge(clk) then
      w_ack <= '0'; -- default
      case state is

        when S_IDLE => -- idle, wait for access request
        -- ------------------------------------------------------------
          if (xbus_cyc_i = '1') and (xbus_stb_i = '1') then
            if (xbus_cti_i = "000") or (xbus_cti_i = "001") then -- single transfer / AMO operation (RMW)
              if (xbus_we_i = '0') then
                state <= S_SINGLE_READ;
              else
                state <= S_SINGLE_WRITE;
              end if;
            elsif BURST_EN and (xbus_cti_i = "010") then -- incrementing address burst
              if (xbus_we_i = '0') then
                state <= S_BURST_READ;
              else
                w_ack <= '1'; -- ACK write-burst start request
                state <= S_BURST_WRITE;
              end if;
            end if;
          end if;

        when S_SINGLE_READ => -- single read in progress
        -- ------------------------------------------------------------
          if (m_axi_rvalid = '1') then
            state <= S_IDLE;
          end if;

        when S_SINGLE_WRITE => -- single write in progress
        -- ------------------------------------------------------------
          if (m_axi_bvalid = '1') then
            state <= S_IDLE;
          end if;

        when S_BURST_READ => -- read burst in progress
        -- ------------------------------------------------------------
          if (m_axi_rvalid = '1') and (m_axi_rlast = '1') then
            state <= S_BURST_END;
          end if;

        when S_BURST_WRITE => -- write burst in progress
        -- ------------------------------------------------------------
          if (xbus_cyc_i = '1') and (xbus_stb_i = '1') and (xbus_cti_i = "010") then
            w_ack <= '1'; -- issue (BURST_LEN/4)-1 local ACKs
          end if;
          if (m_axi_bvalid = '1') then -- this will also issue the remaining last ACK
            state <= S_BURST_END;
          end if;

        when S_BURST_END => -- wait for host-side burst completion
        -- ------------------------------------------------------------
          if (xbus_cti_i = "000") then
            state <= S_IDLE;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          state <= S_IDLE;

      end case;
    end if;
  end process;

  -- state decoder --
  busy  <= '0' when (state = S_IDLE) else '1';
  burst <= '1' when (state = S_BURST_READ)   or (state = S_BURST_WRITE) else '0';
  rw    <= '1' when (state = S_SINGLE_WRITE) or (state = S_BURST_WRITE) else '0';

  -- AXI read data channel --
  m_axi_rready <= '1' when (busy = '1') and (rw = '0') else '0'; -- always ready when doing read accesses
  xbus_rd_ack  <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp(1) = '0') else '0'; -- OKAY(00)/EXOKAY(01)
  xbus_rd_err  <= '1' when (m_axi_rvalid = '1') and (m_axi_rresp(1) = '1') else '0'; -- SLVERR(10)/DECERR(11)
  xbus_dat_o   <= std_ulogic_vector(m_axi_rdata);

  -- AXI write response channel --
  m_axi_bready <= '1' when (busy = '1') and (rw = '1') else '0'; -- always ready when doing write accesses
  xbus_wr_ack  <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp(1) = '0') else '0'; -- OKAY(00)/EXOKAY(01)
  xbus_wr_err  <= '1' when (m_axi_bvalid = '1') and (m_axi_bresp(1) = '1') else '0'; -- SLVERR(10)/DECERR(11)

  -- XBUS response --
  xbus_rsp_o.data <= std_ulogic_vector(m_axi_rdata);
  xbus_rsp_o.ack  <= '1' when BURST_EN and (w_ack = '1') else (xbus_rd_ack or xbus_wr_ack);
  xbus_rsp_o.err  <= '0' when BURST_EN and (w_ack = '1') else (xbus_rd_err or xbus_wr_err);

end architecture;
