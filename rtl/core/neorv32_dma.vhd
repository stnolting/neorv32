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
    clk_i     : in  std_ulogic; -- global clock line
    rstn_i    : in  std_ulogic; -- global reset line, low-active, async
    bus_req_i : in  bus_req_t;  -- bus request
    bus_rsp_o : out bus_rsp_t;  -- bus response
    dma_req_o : out bus_req_t;  -- DMA request
    dma_rsp_i : in  bus_rsp_t;  -- DMA response
    firq_i    : in  std_ulogic_vector(15 downto 0); -- CPU FIRQ channels
    irq_o     : out std_ulogic  -- transfer done interrupt
  );
end neorv32_dma;

architecture neorv32_dma_rtl of neorv32_dma is

  -- transfer type register bits --
  constant type_num_lo_c  : natural :=  0; -- r/w: Number of elements to transfer, LSB
  constant type_num_hi_c  : natural := 23; -- r/w: Number of elements to transfer, MSB
  --
  constant type_qsel_lo_c : natural := 27; -- r/w: Data quantity select, LSB, see below
  constant type_qsel_hi_c : natural := 28; -- r/w: Data quantity select, MSB, see below
  constant type_src_inc_c : natural := 29; -- r/w: SRC constant (0) or incrementing (1) address
  constant type_dst_inc_c : natural := 30; -- r/w: DST constant (0) or incrementing (1) address
  constant type_endian_c  : natural := 31; -- r/w: Convert Endianness when set

  -- control and status register bits --
  constant ctrl_en_c            : natural :=  0; -- r/w: DMA enable
  constant ctrl_auto_c          : natural :=  1; -- r/w: enable FIRQ-triggered transfer
  --
  constant ctrl_error_rd_c      : natural :=  8; -- r/-: error during read transfer
  constant ctrl_error_wr_c      : natural :=  9; -- r/-: error during write transfer
  constant ctrl_busy_c          : natural := 10; -- r/-: DMA transfer in progress
  --
  constant ctrl_firq_mask_lsb_c : natural := 16; -- r/w: FIRQ trigger mask LSB
  constant ctrl_firq_mask_msb_c : natural := 31; -- r/w: FIRQ trigger mask MSB

  -- transfer quantities --
  constant qsel_b2b_c  : std_ulogic_vector(1 downto 0) := "00"; -- byte to byte
  constant qsel_b2uw_c : std_ulogic_vector(1 downto 0) := "01"; -- byte to unsigned word
  constant qsel_b2sw_c : std_ulogic_vector(1 downto 0) := "10"; -- byte to signed word
  constant qsel_w2w_c  : std_ulogic_vector(1 downto 0) := "11"; -- word to word

  -- configuration registers --
  type config_t is record
    enable    : std_ulogic; -- DMA enabled when set
    auto      : std_ulogic; -- FIRQ-driven auto transfer
    firq_mask : std_ulogic_vector(15 downto 0); -- FIRQ trigger mask
    src_base  : std_ulogic_vector(31 downto 0); -- source base address
    dst_base  : std_ulogic_vector(31 downto 0); -- destination base address
    num       : std_ulogic_vector(23 downto 0); -- number of elements
    qsel      : std_ulogic_vector(01 downto 0); -- data quantity select
    src_inc   : std_ulogic; -- constant (0) or incrementing (1) source address
    dst_inc   : std_ulogic; -- constant (0) or incrementing (1) destination address
    endian    : std_ulogic; -- convert endianness when set
    start     : std_ulogic; -- transfer start trigger
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

  -- data alignment --
  signal align_buf : std_ulogic_vector(31 downto 0);
  signal align_end : std_ulogic_vector(31 downto 0);

  -- FIRQ trigger --
  signal firq_buf : std_ulogic_vector(15 downto 0);
  signal match    : std_ulogic;
  signal match_ff : std_ulogic;
  signal atrigger : std_ulogic;

begin

  -- Control Interface -------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------

  -- write access --
  write_access: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      config.enable    <= '0';
      config.auto      <= '0';
      config.firq_mask <= (others => '0');
      config.src_base  <= (others => '0');
      config.dst_base  <= (others => '0');
      config.num       <= (others => '0');
      config.qsel      <= (others => '0');
      config.src_inc   <= '0';
      config.dst_inc   <= '0';
      config.endian    <= '0';
      config.start     <= '0';
    elsif rising_edge(clk_i) then
      config.start <= '0'; -- default
      if (bus_req_i.we = '1') then
        if (bus_req_i.addr(3 downto 2) = "00") then -- control and status register
          config.enable    <= bus_req_i.data(ctrl_en_c);
          config.auto      <= bus_req_i.data(ctrl_auto_c);
          config.firq_mask <= bus_req_i.data(ctrl_firq_mask_msb_c downto ctrl_firq_mask_lsb_c);
        end if;
        if (bus_req_i.addr(3 downto 2) = "01") then -- source base address
          config.src_base <= bus_req_i.data;
        end if;
        if (bus_req_i.addr(3 downto 2) = "10") then -- destination base address
          config.dst_base <= bus_req_i.data;
        end if;
        if (bus_req_i.addr(3 downto 2) = "11") then -- transfer type register
          config.num     <= bus_req_i.data(type_num_hi_c downto type_num_lo_c);
          config.qsel    <= bus_req_i.data(type_qsel_hi_c downto type_qsel_lo_c);
          config.src_inc <= bus_req_i.data(type_src_inc_c);
          config.dst_inc <= bus_req_i.data(type_dst_inc_c);
          config.endian  <= bus_req_i.data(type_endian_c);
          config.start   <= '1'; -- trigger DMA operation
        end if;
      end if;
    end if;
  end process write_access;

  -- read access --
  read_access: process(clk_i)
  begin
    if rising_edge(clk_i) then
      bus_rsp_o.ack  <= bus_req_i.re or bus_req_i.we; -- bus access acknowledge
      bus_rsp_o.data <= (others => '0');
      if (bus_req_i.re = '1') then
        case bus_req_i.addr(3 downto 2) is
          when "00" => -- control and status register
            bus_rsp_o.data(ctrl_en_c)       <= config.enable;
            bus_rsp_o.data(ctrl_auto_c)     <= config.auto;
            bus_rsp_o.data(ctrl_error_rd_c) <= engine.err_rd;
            bus_rsp_o.data(ctrl_error_wr_c) <= engine.err_wr;
            bus_rsp_o.data(ctrl_busy_c)     <= engine.busy;
            bus_rsp_o.data(ctrl_firq_mask_msb_c downto ctrl_firq_mask_lsb_c) <= config.firq_mask;
          when "01" => -- address of last read access
            bus_rsp_o.data <= engine.src_addr;
          when "10" => -- address of last write access
            bus_rsp_o.data <= engine.dst_addr;
          when others => -- transfer type register
            bus_rsp_o.data(type_num_hi_c downto type_num_lo_c)   <= engine.num;
            bus_rsp_o.data(type_qsel_hi_c downto type_qsel_lo_c) <= config.qsel;
            bus_rsp_o.data(type_src_inc_c)                       <= config.src_inc;
            bus_rsp_o.data(type_dst_inc_c)                       <= config.dst_inc;
            bus_rsp_o.data(type_endian_c)                        <= config.endian;
        end case;
      end if;
    end if;
  end process read_access;

  -- no access error possible --
  bus_rsp_o.err <= '0';


  -- Automatic Trigger ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  automatic_trigger: process(clk_i)
  begin
    if rising_edge(clk_i) then
      firq_buf <= firq_i;
      match_ff <= match;
      atrigger <= match and (not match_ff); -- trigger on rising edge of FIRQ
    end if;
  end process automatic_trigger;

  -- logical OR of all enabled trigger FIRQs --
  match <= or_reduce_f(firq_buf and config.firq_mask);


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
      dma_req_o.re    <= '0';
      dma_req_o.we    <= '0';
    elsif rising_edge(clk_i) then
      -- defaults --
      engine.done  <= '0';
      dma_req_o.re <= '0';
      dma_req_o.we <= '0';

      -- state machine --
      case engine.state is

        when S_IDLE => -- idle, waiting for start trigger
        -- ------------------------------------------------------------
          engine.src_addr <= config.src_base;
          engine.dst_addr <= config.dst_base;
          engine.num      <= config.num;
          if (config.enable = '1') and
             (((config.auto = '0') and (config.start = '1')) or -- manual trigger
              ((config.auto = '1') and (atrigger = '1'))) then -- automatic trigger
            engine.err_rd <= '0';
            engine.err_wr <= '0';
            dma_req_o.re  <= '1'; -- issue read request
            engine.state  <= S_READ;
          end if;

        when S_READ => -- pending read access
        -- ------------------------------------------------------------
          if (dma_rsp_i.err = '1') then
            engine.done   <= '1';
            engine.err_rd <= '1';
            engine.state  <= S_IDLE;
          elsif (dma_rsp_i.ack = '1') then
            dma_req_o.we <= '1';
            engine.state <= S_WRITE;
          end if;

        when S_WRITE => -- pending write access
        -- ------------------------------------------------------------
          if (dma_rsp_i.err = '1') then
            engine.done   <= '1';
            engine.err_wr <= '1';
            engine.state  <= S_IDLE;
          elsif (dma_rsp_i.ack = '1') then
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
            dma_req_o.re <= '1'; -- issue read request
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
  irq_o <= engine.done and config.enable; -- no interrupt if transfer was aborted

  -- bus output --
  dma_req_o.priv <= priv_mode_m_c;
  dma_req_o.src  <= '0'; -- source = data access
  dma_req_o.addr <= engine.src_addr when (engine.state = S_READ) else engine.dst_addr;
  dma_req_o.rvso <= '0'; -- no reservation set operation possible

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
  align_end <= dma_rsp_i.data when (config.endian = '0') else bswap32_f(dma_rsp_i.data);

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
    dma_req_o.ben <= (others => '0'); -- default
    if (config.qsel = qsel_b2b_c) then -- byte
      dma_req_o.data(07 downto 00) <= align_buf(7 downto 0);
      dma_req_o.data(15 downto 08) <= align_buf(7 downto 0);
      dma_req_o.data(23 downto 16) <= align_buf(7 downto 0);
      dma_req_o.data(31 downto 24) <= align_buf(7 downto 0);
      dma_req_o.ben(to_integer(unsigned(engine.dst_addr(1 downto 0)))) <= '1';
    else -- word
      dma_req_o.data <= align_buf;
      dma_req_o.ben  <= "1111";
    end if;
  end process dst_align;


end neorv32_dma_rtl;
