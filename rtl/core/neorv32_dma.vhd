-- #################################################################################################
-- # << NEORV32 - Direct Memory Access (DMA) Controller >>                                         #
-- # ********************************************************************************************* #
-- # Simple single-channel scatter/gather DMA controller that is also capable of transforming data #
-- # while moving it from source to destination.                                                   #
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

entity neorv32_dma is
  port (
    -- global control --
    clk_i          : in  std_ulogic; -- global clock line
    rstn_i         : in  std_ulogic; -- global reset line, low-active, async
    -- peripheral port: configuration and status --
    addr_i         : in  std_ulogic_vector(31 downto 0); -- address
    rden_i         : in  std_ulogic; -- read enable
    wren_i         : in  std_ulogic; -- write enable
    data_i         : in  std_ulogic_vector(31 downto 0); -- data in
    data_o         : out std_ulogic_vector(31 downto 0); -- data out
    ack_o          : out std_ulogic; -- transfer acknowledge
    -- host port: bus access --
    bus_bus_priv_o : out std_ulogic; -- current privilege level
    bus_cached_o   : out std_ulogic; -- set if cached (!) access in progress
    bus_src_o      : out std_ulogic; -- access source
    bus_addr_o     : out std_ulogic_vector(31 downto 0); -- bus access address
    bus_rdata_i    : in  std_ulogic_vector(31 downto 0); -- bus read data
    bus_wdata_o    : out std_ulogic_vector(31 downto 0); -- bus write data
    bus_ben_o      : out std_ulogic_vector(03 downto 0); -- byte enable
    bus_we_o       : out std_ulogic; -- write enable
    bus_re_o       : out std_ulogic; -- read enable
    bus_ack_i      : in  std_ulogic; -- bus transfer acknowledge
    bus_err_i      : in  std_ulogic; -- bus transfer error
    -- interrupt --
    irq_o          : out std_ulogic
  );
end neorv32_dma;

architecture neorv32_dma_rtl of neorv32_dma is

  -- IO space: module base address --
  constant hi_abb_c : natural := index_size_f(io_size_c)-1; -- high address boundary bit
  constant lo_abb_c : natural := index_size_f(dma_size_c); -- low address boundary bit

  -- control access control --
  signal acc_en : std_ulogic; -- module access enable
  signal wren   : std_ulogic; -- word write enable
  signal rden   : std_ulogic; -- read enable

  -- transfer type register bits --
  constant type_num_lo_c  : natural :=  0; -- r/w: Number of elements to transfer, LSB
  constant type_num_hi_c  : natural := 23; -- r/w: Number of elements to transfer, MSB

  constant type_qsel_lo_c : natural := 27; -- r/w: Data quantity select, LSB, see below
  constant type_qsel_hi_c : natural := 28; -- r/w: Data quantity select, MSB, see below
  constant type_src_inc_c : natural := 29; -- r/w: SRC constant (0) or incrementing (1) address
  constant type_dst_inc_c : natural := 30; -- r/w: DST constant (0) or incrementing (1) address
  constant type_endian_c  : natural := 31; -- r/w: Convert Endianness when set

  -- control and status register bits --
  constant ctrl_en_c       : natural :=  0; -- r/w: DMA enable
  constant ctrl_error_rd_c : natural := 29; -- r/-: error during read transfer
  constant ctrl_error_wr_c : natural := 30; -- r/-: error during write transfer
  constant ctrl_busy_c     : natural := 31; -- r/-: DMA transfer in progress

  -- transfer quantities --
  constant qsel_b2b_c  : std_ulogic_vector(1 downto 0) := "00"; -- byte to byte
  constant qsel_b2uw_c : std_ulogic_vector(1 downto 0) := "01"; -- byte to unsigned word
  constant qsel_b2sw_c : std_ulogic_vector(1 downto 0) := "10"; -- byte to signed word
  constant qsel_w2w_c  : std_ulogic_vector(1 downto 0) := "11"; -- word to word

  -- configuration registers --
  type config_t is record
    enable   : std_ulogic; -- DMA enabled when set
    src_base : std_ulogic_vector(31 downto 0); -- source base address
    dst_base : std_ulogic_vector(31 downto 0); -- destination base address
    num      : std_ulogic_vector(23 downto 0); -- number of elements
    qsel     : std_ulogic_vector(01 downto 0); -- data quantity select
    src_inc  : std_ulogic; -- constant (0) or incrementing (1) source address
    dst_inc  : std_ulogic; -- constant (0) or incrementing (1) destination address
    endian   : std_ulogic; -- convert endianness when set
    start    : std_ulogic; -- transfer start trigger
  end record;
  signal config : config_t;

  -- bus access engine --
  type state_t is (S_IDLE, S_READ, S_WRITE, S_NEXT);
  type engine_t is record
    state    : state_t;
    src_addr : std_ulogic_vector(31 downto 0);
    dst_addr : std_ulogic_vector(31 downto 0);
    num      : std_ulogic_vector(23 downto 0);
    err_rd   : std_ulogic;
    err_wr   : std_ulogic;
    src_add  : unsigned(31 downto 0);
    dst_add  : unsigned(31 downto 0);
    busy     : std_ulogic;
    done     : std_ulogic;
  end record;
  signal engine : engine_t;

  -- data aligner --
  signal align_buf : std_ulogic_vector(31 downto 0);
  signal align_end : std_ulogic_vector(31 downto 0);
  signal align_tmp : std_ulogic_vector(31 downto 0);

begin

  -- Control Interface -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- access control --
  acc_en <= '1' when (addr_i(hi_abb_c downto lo_abb_c) = dma_base_c(hi_abb_c downto lo_abb_c)) else '0';
  wren   <= acc_en and wren_i;
  rden   <= acc_en and rden_i;

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      config.enable   <= '0';
      config.src_base <= (others => '0');
      config.dst_base <= (others => '0');
      config.num      <= (others => '0');
      config.qsel     <= (others => '0');
      config.src_inc  <= '0';
      config.dst_inc  <= '0';
      config.endian   <= '0';
      config.start    <= '0';
    elsif rising_edge(clk_i) then
      config.start <= '0'; -- default
      if (wren = '1') then
        if (addr_i(3 downto 2) = "00") then -- control and status register
          config.enable <= data_i(ctrl_en_c);
        end if;
        if (addr_i(3 downto 2) = "01") then -- source base address
          config.src_base <= data_i;
        end if;
        if (addr_i(3 downto 2) = "10") then -- destination base address
          config.dst_base <= data_i;
        end if;
        if (addr_i(3 downto 2) = "11") then -- transfer type register
          config.num     <= data_i(type_num_hi_c downto type_num_lo_c);
          config.qsel    <= data_i(type_qsel_hi_c downto type_qsel_lo_c);
          config.src_inc <= data_i(type_src_inc_c);
          config.dst_inc <= data_i(type_dst_inc_c);
          config.endian  <= data_i(type_endian_c);
          config.start   <= '1'; -- trigger DMA operation
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      ack_o  <= rden or wren; -- bus access acknowledge
      data_o <= (others => '0');
      if (rden = '1') then
        case addr_i(3 downto 2) is
          when "00" => -- control and status register
            data_o(ctrl_en_c)       <= config.enable;
            data_o(ctrl_error_rd_c) <= engine.err_rd;
            data_o(ctrl_error_wr_c) <= engine.err_wr;
            data_o(ctrl_busy_c)     <= engine.busy;
          when "01" => -- address of last read access
            data_o <= engine.src_addr;
          when "10" => -- address of last write access
            data_o <= engine.dst_addr;
          when others => -- transfer type register
            data_o(type_num_hi_c downto type_num_lo_c)   <= engine.num;
            data_o(type_qsel_hi_c downto type_qsel_lo_c) <= config.qsel;
            data_o(type_src_inc_c)                       <= config.src_inc;
            data_o(type_dst_inc_c)                       <= config.dst_inc;
            data_o(type_endian_c)                        <= config.endian;
        end case;
      end if;
    end if;
  end process read_access;


  -- Bus Access Engine ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  bus_engine: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      engine.state    <= S_IDLE;
      engine.src_addr <= (others => '0');
      engine.dst_addr <= (others => '0');
      engine.num      <= (others => '0');
      engine.err_rd   <= '0';
      engine.err_wr   <= '0';
      engine.done     <= '0';
      bus_re_o        <= '0';
      bus_we_o        <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      engine.done <= '0';
      bus_re_o    <= '0';
      bus_we_o    <= '0';

      -- state machine --
      case engine.state is

        when S_IDLE => -- idle, waiting for start trigger
        -- ------------------------------------------------------------
          engine.src_addr <= config.src_base;
          engine.dst_addr <= config.dst_base;
          engine.num      <= config.num;
          if (config.enable = '1') and (config.start = '1') then
            engine.err_rd <= '0';
            engine.err_wr <= '0';
            bus_re_o      <= '1'; -- issue read request
            engine.state  <= S_READ;
          end if;

        when S_READ => -- pending read access
        -- ------------------------------------------------------------
          if (bus_err_i = '1') then
            engine.done   <= '1';
            engine.err_rd <= '1';
            engine.state  <= S_IDLE;
          elsif (bus_ack_i = '1') then
            bus_we_o     <= '1';
            engine.state <= S_WRITE;
          end if;

        when S_WRITE => -- pending write access
        -- ------------------------------------------------------------
          if (bus_err_i = '1') then
            engine.done   <= '1';
            engine.err_wr <= '1';
            engine.state  <= S_IDLE;
          elsif (bus_ack_i = '1') then
            engine.num   <= std_ulogic_vector(unsigned(engine.num) - 1);
            engine.state <= S_NEXT;
          end if;

        when S_NEXT => -- check if done; prepare next access
        -- ------------------------------------------------------------
          if (or_reduce_f(engine.num) = '0') or (config.enable = '0') then -- transfer done or aborted?
            engine.done  <= '1';
            engine.state <= S_IDLE;
          else
            if (config.src_inc = '1') then -- incrementing source address
              engine.src_addr <= std_ulogic_vector(unsigned(engine.src_addr) + engine.src_add);
            end if;
            if (config.dst_inc = '1') then -- incrementing destination address
              engine.dst_addr <= std_ulogic_vector(unsigned(engine.dst_addr) + engine.dst_add);
            end if;
            bus_re_o     <= '1'; -- issue read request
            engine.state <= S_READ;
          end if;

        when others => -- undefined
        -- ------------------------------------------------------------
          engine.state <= S_IDLE;

      end case;
    end if;
  end process bus_engine;

  -- transfer in progress? --
  engine.busy <= '0' when (engine.state = S_IDLE) else '1';

  -- transfer-done interrupt --
  irq_o <= engine.done and config.enable;

  -- bus output --
  bus_bus_priv_o <= priv_mode_m_c;
  bus_cached_o   <= '0';
  bus_src_o      <= '0'; -- data access
  bus_addr_o     <= engine.src_addr when (engine.state = S_READ) else engine.dst_addr;

  -- address increment --
  address_inc: process(config.qsel)
  begin
    case config.qsel is
      when qsel_b2b_c => engine.src_add <= to_unsigned(1, 32); engine.dst_add <= to_unsigned(1, 32); -- byte -> byte
      when qsel_w2w_c => engine.src_add <= to_unsigned(4, 32); engine.dst_add <= to_unsigned(4, 32); -- word -> word
      when others     => engine.src_add <= to_unsigned(1, 32); engine.dst_add <= to_unsigned(4, 32); -- byte -> word
    end case;
  end process address_inc;


  -- Data Transformer -----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- endianness conversion --
  align_end <= bus_rdata_i when (config.endian = '0') else bswap32_f(bus_rdata_i);

  -- source data alignment --
  src_align: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      align_buf <= (others => '0');
    elsif rising_edge(clk_i) then
      if (engine.state = S_READ) then
        if (config.qsel = qsel_w2w_c) then -- word
          align_buf <= align_end;
        else -- byte
          case engine.src_addr(1 downto 0) is
            when "00" => -- byte 0
              align_buf(07 downto 0) <= align_end(07 downto 00);
              align_buf(31 downto 8) <= (others => (config.qsel(1) and align_end(07))); -- sign extension
            when "01" => -- byte 1
              align_buf(07 downto 0) <= align_end(15 downto 08);
              align_buf(31 downto 8) <= (others => (config.qsel(1) and align_end(15))); -- sign extension
            when "10" => -- byte 2
              align_buf(07 downto 0) <= align_end(23 downto 16);
              align_buf(31 downto 8) <= (others => (config.qsel(1) and align_end(23))); -- sign extension
            when others => -- byte 3
              align_buf(07 downto 0) <= align_end(31 downto 24);
              align_buf(31 downto 8) <= (others => (config.qsel(1) and align_end(31))); -- sign extension
          end case;
        end if;
      end if;
    end if;
  end process src_align;

  -- destination data alignment --
  dst_align: process(config.qsel, align_buf, engine.dst_addr)
  begin
    bus_ben_o <= (others => '0'); -- default
    if (config.qsel = qsel_b2b_c) then -- byte
      bus_wdata_o(07 downto 00) <= align_buf(7 downto 0);
      bus_wdata_o(15 downto 08) <= align_buf(7 downto 0);
      bus_wdata_o(23 downto 16) <= align_buf(7 downto 0);
      bus_wdata_o(31 downto 24) <= align_buf(7 downto 0);
      bus_ben_o(to_integer(unsigned(engine.dst_addr(1 downto 0)))) <= '1';
    else -- word
      bus_wdata_o <= align_buf;
      bus_ben_o   <= "1111";
    end if;
  end process dst_align;


end neorv32_dma_rtl;
