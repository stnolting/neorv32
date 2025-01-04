-- ================================================================================ --
-- NEORV32 CPU - Inter-Core Communication Unit (ICC)                                --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity neorv32_cpu_icc is
  generic (
    HART_ID   : natural range 0 to 3; -- ID of this core
    NUM_HARTS : natural range 1 to 4 -- number of cores, has to be a power of two
  );
  port (
    -- global control --
    clk_i        : in  std_ulogic; -- global clock, rising edge
    rstn_i       : in  std_ulogic; -- global reset, low-active, async
    -- CSR interface --
    csr_we_i     : in  std_ulogic; -- global write enable
    csr_re_i     : in  std_ulogic; -- global read enable
    csr_addr_i   : in  std_ulogic_vector(11 downto 0); -- address
    csr_wdata_i  : in  std_ulogic_vector(XLEN-1 downto 0); -- write data
    csr_rdata_o  : out std_ulogic_vector(XLEN-1 downto 0); -- read data
    -- ICC TX links --
    icc_tx_rdy_o : out std_ulogic_vector(NUM_HARTS-1 downto 0); -- data available
    icc_tx_ack_i : in  std_ulogic_vector(NUM_HARTS-1 downto 0); -- read-enable
    icc_tx_dat_o : out std_ulogic_vector((NUM_HARTS*XLEN)-1 downto 0); -- data word
    -- ICC RX links --
    icc_rx_rdy_i : in  std_ulogic_vector(NUM_HARTS-1 downto 0); -- data available
    icc_rx_ack_o : out std_ulogic_vector(NUM_HARTS-1 downto 0); -- read-enable
    icc_rx_dat_i : in  std_ulogic_vector((NUM_HARTS*XLEN)-1 downto 0) -- data word
  );
end neorv32_cpu_icc;

architecture neorv32_cpu_icc_rtl of neorv32_cpu_icc is

  -- link select --
  constant id_width_c : natural := index_size_f(NUM_HARTS);
  signal link_id : std_ulogic_vector(id_width_c-1 downto 0);

  -- link control --
  signal link_sel, tx_fifo_we, tx_fifo_free : std_ulogic_vector(NUM_HARTS-1 downto 0);

  -- incoming data as array --
  type rx_data_t is array (0 to NUM_HARTS-1) of std_ulogic_vector(XLEN-1 downto 0);
  signal rx_data : rx_data_t;

begin

  -- CSR Access -----------------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  csr_write: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      link_id <= (others => '0');
    elsif rising_edge(clk_i) then
      if (csr_we_i = '1') and (csr_addr_i(11 downto 1) = csr_mxiccsr0_c(11 downto 1)) then
        link_id <= csr_wdata_i(id_width_c-1 downto 0);
      end if;
    end if;
  end process csr_write;

  csr_read: process(csr_addr_i, link_id, icc_rx_rdy_i, tx_fifo_free, rx_data)
  begin
    csr_rdata_o <= (others => '0'); -- default
    if (csr_addr_i(11 downto 2) = csr_mxiccrxd_c(11 downto 2)) then -- ICC CSRs base address
      if (csr_addr_i(1) = '0') then -- data register(s)
        csr_rdata_o <= rx_data(to_integer(unsigned(link_id)));
      else -- control and status register(s)
        csr_rdata_o(XLEN-1)                <= icc_rx_rdy_i(to_integer(unsigned(link_id)));
        csr_rdata_o(XLEN-2)                <= tx_fifo_free(to_integer(unsigned(link_id)));
        csr_rdata_o(id_width_c-1 downto 0) <= link_id;
      end if;
    end if;
  end process csr_read;


  -- Communication Links --------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  link_gen:
  for i in 0 to NUM_HARTS-1 generate

    -- TX FIFOs for outgoing links --
    queue_gen:
    if i /= HART_ID generate
      queue_inst: entity neorv32.neorv32_fifo
      generic map (
        FIFO_DEPTH => 4, -- yes, this is fixed
        FIFO_WIDTH => XLEN,
        FIFO_RSYNC => true,
        FIFO_SAFE  => true,
        FULL_RESET => true
      )
      port map (
        -- control --
        clk_i   => clk_i,
        rstn_i  => rstn_i,
        clear_i => '0',
        half_o  => open,
        -- write port --
        wdata_i => csr_wdata_i,
        we_i    => tx_fifo_we(i),
        free_o  => tx_fifo_free(i),
        -- read port --
        re_i    => icc_tx_ack_i(i),
        rdata_o => icc_tx_dat_o(i*XLEN+(XLEN-1) downto i*XLEN),
        avail_o => icc_tx_rdy_o(i)
      );
    end generate;

    -- no FIFO/link for *this* core --
    queue_terminate:
    if i = HART_ID generate
      tx_fifo_free(i) <= '0';
      icc_tx_dat_o(i*XLEN+(XLEN-1) downto i*XLEN) <= (others => '0');
      icc_tx_rdy_o(i) <= '0';
    end generate;

    -- reorganize incoming links as 2d-array --
    rx_data(i) <= icc_rx_dat_i(i*XLEN+(XLEN-1) downto i*XLEN);

    -- link control --
    link_sel(i)     <= '1' when (unsigned(link_id) = to_unsigned(i, id_width_c)) else '0';
    icc_rx_ack_o(i) <= '1' when (csr_re_i = '1') and (csr_addr_i = csr_mxiccrxd_c) and (link_sel(i) = '1') else '0';
    tx_fifo_we(i)   <= '1' when (csr_we_i = '1') and (csr_addr_i = csr_mxicctxd_c) and (link_sel(i) = '1') else '0';

  end generate;


end neorv32_cpu_icc_rtl;
