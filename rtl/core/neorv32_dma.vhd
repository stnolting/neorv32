-- ================================================================================ --
-- NEORV32 SoC - Direct Memory Access Controller (DMA)                              --
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

entity neorv32_dma is
  generic (
    DSC_FIFO : natural range 4 to 512 := 4 -- descriptor FIFO depth (1 descriptor = 3 entries)
  );
  port (
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    dma_req_o : out bus_req_t;  -- DMA request
    dma_rsp_i : in  bus_rsp_t;  -- DMA response
    irq_o     : out std_ulogic  -- transfer done interrupt
  );
end neorv32_dma;

architecture neorv32_dma_rtl of neorv32_dma is

  -- FIFO size helper --
  constant log2_fifo_size_c : natural := index_size_f(DSC_FIFO); -- extend to next power of two
  constant fifo_size_c      : natural := 2**log2_fifo_size_c; -- actual FIFO size

  -- transfer configuration (part of the descriptor) --
  constant conf_num_lo_c : natural :=  0; -- r/w: number of elements to transfer, LSB
  constant conf_num_hi_c : natural := 23; -- r/w: number of elements to transfer, MSB
  constant conf_bswap_c  : natural := 27; -- r/w: swap byte order
  constant conf_src_lo_c : natural := 28; -- r/w: source addressing (0=byte, 1=word)
  constant conf_src_hi_c : natural := 29; -- r/w: source addressing (0=const, 1=inc)
  constant conf_dst_lo_c : natural := 30; -- r/w: destination addressing (0=byte, 1=word)
  constant conf_dst_hi_c : natural := 31; -- r/w: destination addressing (0=const, 1=inc)

  -- control and status register bits --
  constant ctrl_en_c     : natural :=  0; -- r/w: DMA enable
  constant ctrl_start_c  : natural :=  1; -- -/w: start DMA transfer(s)
  constant ctrl_fifo0_c  : natural := 16; -- r/-: log2(FIFO descriptor depth), LSB
  constant ctrl_fifo3_c  : natural := 19; -- r/-: log2(FIFO descriptor depth), MSB
  constant ctrl_dempty_c : natural := 27; -- r/-: descriptor buffer is empty
  constant ctrl_dfull_c  : natural := 28; -- r/-: descriptor buffer is full
  constant ctrl_error_c  : natural := 29; -- r/-: bus access error during transfer
  constant ctrl_done_c   : natural := 30; -- r/c: transfer has completed
  constant ctrl_busy_c   : natural := 31; -- r/-: DMA transfer in progress

  -- control and status register --
  type ctrl_t is record
    enable, start, err, done : std_ulogic;
  end record;
  signal ctrl : ctrl_t;

  -- descriptor FIFO interface --
  type fifo_t is record
    clr   : std_ulogic;
    we    : std_ulogic;
    re    : std_ulogic;
    rdata : std_ulogic_vector(31 downto 0);
    avail : std_ulogic;
    free  : std_ulogic;
  end record;
  signal fifo : fifo_t;

  -- bus access engine --
  type state_t is (S_IDLE, S_GET_0, S_GET_1, S_READ_REQ, S_READ_RSP, S_WRITE_REQ, S_WRITE_RSP, S_NEXT);
  type engine_t is record
    state    : state_t;
    run      : std_ulogic;
    run_ff   : std_ulogic;
    done     : std_ulogic;
    err      : std_ulogic;
    src_add  : unsigned(31 downto 0);
    dst_add  : unsigned(31 downto 0);
    src_addr : std_ulogic_vector(31 downto 0);
    dst_addr : std_ulogic_vector(31 downto 0);
    num      : std_ulogic_vector(23 downto 0);
    bswap    : std_ulogic; -- swap byte order
    src_type : std_ulogic_vector(1 downto 0);
    dst_type : std_ulogic_vector(1 downto 0);
  end record;
  signal engine : engine_t;

  -- data buffer --
  signal rdata, data_buf : std_ulogic_vector(31 downto 0);

begin

  -- Control and Status Register ------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  ctrl_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      bus_rsp_o   <= rsp_terminate_c;
      ctrl.enable <= '0';
      ctrl.start  <= '0';
      ctrl.err    <= '0';
      ctrl.done   <= '0';
    elsif rising_edge(clk_i) then
      -- bus handshake --
      bus_rsp_o.ack  <= bus_req_i.stb;
      bus_rsp_o.err  <= '0';
      bus_rsp_o.data <= (others => '0');
      -- defaults --
      ctrl.start <= '0';
      ctrl.err   <= ctrl.enable and (ctrl.err  or engine.err);
      ctrl.done  <= ctrl.enable and (ctrl.done or engine.done);
      -- bus access --
      if (bus_req_i.stb = '1') and (bus_req_i.addr(2) = '0') then
        if (bus_req_i.rw = '1') then -- write access
          ctrl.enable <= bus_req_i.data(ctrl_en_c);
          ctrl.start  <= bus_req_i.data(ctrl_start_c);
          ctrl.err    <= ctrl.err and (not bus_req_i.data(ctrl_error_c)); -- write 1 to clear
          ctrl.done   <= ctrl.done and (not bus_req_i.data(ctrl_done_c)); -- write 1 to clear
        else -- read access
          bus_rsp_o.data(ctrl_en_c)     <= ctrl.enable;
          bus_rsp_o.data(ctrl_fifo3_c downto ctrl_fifo0_c) <= std_ulogic_vector(to_unsigned(log2_fifo_size_c, 4));
          bus_rsp_o.data(ctrl_dempty_c) <= not fifo.avail;
          bus_rsp_o.data(ctrl_dfull_c)  <= not fifo.free;
          bus_rsp_o.data(ctrl_error_c)  <= ctrl.err;
          bus_rsp_o.data(ctrl_done_c)   <= ctrl.done;
          bus_rsp_o.data(ctrl_busy_c)   <= engine.run;
        end if;
      end if;
    end if;
  end process ctrl_access;

  -- transfer-done interrupt --
  irq_o <= ctrl.done;


  -- Descriptor Buffer (FIFO) ---------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  descriptor_buffer: entity neorv32.neorv32_fifo
  generic map (
    FIFO_DEPTH => fifo_size_c,
    FIFO_WIDTH => 32,
    FIFO_RSYNC => false,
    FIFO_SAFE  => true,
    FULL_RESET => false
  )
  port map (
    -- control and status --
    clk_i   => clk_i,
    rstn_i  => rstn_i,
    clear_i => fifo.clr,
    half_o  => open,
    level_o => open,
    -- write port --
    wdata_i => bus_req_i.data,
    we_i    => fifo.we,
    free_o  => fifo.free,
    -- read port --
    re_i    => fifo.re,
    rdata_o => fifo.rdata,
    avail_o => fifo.avail
  );

  -- FIFO control --
  fifo.clr <= '1' when (ctrl.enable = '0') else '0';
  fifo.we  <= '1' when (bus_req_i.stb = '1') and (bus_req_i.rw = '1') and (bus_req_i.addr(2) = '1') else '0';
  fifo.re  <= engine.run when (engine.state = S_IDLE) or (engine.state = S_GET_0) or (engine.state = S_GET_1) else '0';


  -- Bus Access Engine ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      engine.state    <= S_IDLE;
      engine.run      <= '0';
      engine.run_ff   <= '0';
      engine.err      <= '0';
      engine.src_addr <= (others => '0');
      engine.dst_addr <= (others => '0');
      engine.num      <= (others => '0');
      engine.bswap    <= '0';
      engine.src_type <= (others => '0');
      engine.dst_type <= (others => '0');
    elsif rising_edge(clk_i) then
      engine.run_ff <= engine.run;
      case engine.state is

        when S_IDLE => -- idle, waiting for trigger
        -- ------------------------------------------------------------
          engine.err <= '0';
          if (engine.run = '1') and (fifo.avail = '1') and (engine.err = '0') then
            engine.src_addr <= fifo.rdata; -- get descriptor: source base address
            engine.state    <= S_GET_0;
          else
            engine.run <= ctrl.enable and ctrl.start and fifo.avail and (not ctrl.err);
          end if;

        when S_GET_0 => -- get descriptor: destination base address
        -- ------------------------------------------------------------
          if (fifo.avail = '1') then
            engine.dst_addr <= fifo.rdata;
            engine.state    <= S_GET_1;
          else
            engine.err   <= '1'; -- error - no descriptor data available
            engine.state <= S_NEXT;
          end if;

        when S_GET_1 => -- get descriptor: transfer configuration
        -- ------------------------------------------------------------
          if (fifo.avail = '1') then
            engine.num      <= fifo.rdata(conf_num_hi_c downto conf_num_lo_c);
            engine.bswap    <= fifo.rdata(conf_bswap_c);
            engine.src_type <= fifo.rdata(conf_src_hi_c downto conf_src_lo_c);
            engine.dst_type <= fifo.rdata(conf_dst_hi_c downto conf_dst_lo_c);
            engine.state    <= S_READ_REQ;
          else
            engine.err   <= '1'; -- error - no descriptor data available
            engine.state <= S_NEXT;
          end if;

        when S_READ_REQ => -- read request
        -- ------------------------------------------------------------
          engine.state <= S_READ_RSP;

        when S_READ_RSP => -- read response
        -- ------------------------------------------------------------
          if (dma_rsp_i.ack = '1') then
            engine.err <= dma_rsp_i.err;
            if (dma_rsp_i.err = '1') then
              engine.state <= S_NEXT;
            else
              engine.state <= S_WRITE_REQ;
            end if;
          end if;

        when S_WRITE_REQ => -- write request
        -- ------------------------------------------------------------
          engine.num   <= std_ulogic_vector(unsigned(engine.num) - 1);
          engine.state <= S_WRITE_RSP;

        when S_WRITE_RSP => -- write response
        -- ------------------------------------------------------------
          if (dma_rsp_i.ack = '1') then
            engine.err   <= dma_rsp_i.err;
            engine.state <= S_NEXT;
          end if;

        when S_NEXT => -- check if done; prepare next access
        -- ------------------------------------------------------------
          engine.src_addr <= std_ulogic_vector(unsigned(engine.src_addr) + engine.src_add);
          engine.dst_addr <= std_ulogic_vector(unsigned(engine.dst_addr) + engine.dst_add);
          if (or_reduce_f(engine.num) = '0') or (ctrl.enable = '0') or (engine.err = '1') then -- all elements done? abort?
            engine.state <= S_IDLE;
          else
            engine.state <= S_READ_REQ;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          engine.state <= S_IDLE;

      end case;
    end if;
  end process bus_engine;

  -- transfer(s) done --
  engine.done <= '1' when (engine.run = '0') and (engine.run_ff = '1') else '0';


  -- Bus Output Control ---------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_control: process(engine, data_buf)
  begin
    dma_req_o <= req_terminate_c; -- all-zero by default
    -- meta data --
    dma_req_o.priv  <= priv_mode_m_c; -- transfers execute with highest privilege level
    dma_req_o.src   <= '0'; -- "data" transfer
    dma_req_o.amo   <= '0'; -- no atomic operations
    dma_req_o.burst <= '0'; -- no burst transfers
    dma_req_o.lock  <= '0'; -- no locked accesses
    -- read/write --
    if (engine.state = S_READ_REQ) or (engine.state = S_READ_RSP) then -- read access
      dma_req_o.addr <= engine.src_addr(31 downto 2) & "00";
      dma_req_o.rw   <= '0';
      if (engine.src_type(0) = '0') then -- byte
        dma_req_o.ben(to_integer(unsigned(engine.src_addr(1 downto 0)))) <= '1';
      else -- word
        dma_req_o.ben <= (others => '1');
      end if;
    else -- write access
      dma_req_o.addr <= engine.dst_addr(31 downto 2) & "00";
      dma_req_o.rw   <= '1';
      if (engine.dst_type(0) = '0') then -- byte
        dma_req_o.ben(to_integer(unsigned(engine.dst_addr(1 downto 0)))) <= '1';
      else -- word
        dma_req_o.ben <= (others => '1');
      end if;
    end if;
    -- output data alignment --
    if (engine.dst_type(0) = '0') then -- byte
      dma_req_o.data <= data_buf(7 downto 0) & data_buf(7 downto 0) & data_buf(7 downto 0) & data_buf(7 downto 0);
    else -- word
      dma_req_o.data <= data_buf;
    end if;
    -- request strobe --
    if (engine.state = S_READ_REQ) or (engine.state = S_WRITE_REQ) then
      dma_req_o.stb <= '1';
    end if;
  end process bus_control;


  -- Address Increment ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  address_inc: process(engine)
  begin
    -- source --
    case engine.src_type is
      when "10"   => engine.src_add <= to_unsigned(1, 32); -- incrementing byte
      when "11"   => engine.src_add <= to_unsigned(4, 32); -- incrementing word
      when others => engine.src_add <= to_unsigned(0, 32); -- constant byte/word
    end case;
    -- destination --
    case engine.dst_type is
      when "10"   => engine.dst_add <= to_unsigned(1, 32); -- incrementing byte
      when "11"   => engine.dst_add <= to_unsigned(4, 32); -- incrementing word
      when others => engine.dst_add <= to_unsigned(0, 32); -- constant byte/word
    end case;
  end process address_inc;


  -- Input Data Alignment -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  src_align: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      data_buf <= (others => '0');
    elsif rising_edge(clk_i) then
      if (engine.state = S_READ_RSP) then
        if (engine.src_type(0) = '0') then -- byte
          case engine.src_addr(1 downto 0) is
            when "00"   => data_buf <= x"000000" & rdata(7 downto 0);
            when "01"   => data_buf <= x"000000" & rdata(15 downto 8);
            when "10"   => data_buf <= x"000000" & rdata(23 downto 16);
            when others => data_buf <= x"000000" & rdata(31 downto 24);
          end case;
        else -- word
          data_buf <= rdata;
        end if;
      end if;
    end if;
  end process src_align;

  -- swap byte order (Endianness conversion) --
  bswap_gen:
  for i in 0 to 3 generate
    rdata(i*8+7 downto i*8) <= dma_rsp_i.data(i*8+7 downto i*8) when (engine.bswap = '0') else
                               dma_rsp_i.data((32-i*8)-1 downto 32-(i+1)*8);
  end generate;


end neorv32_dma_rtl;
