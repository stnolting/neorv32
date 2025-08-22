-- ================================================================================ --
-- NEORV32 - Simple XBUS / Wishbone Memory (meant for simulation only)              --
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
use std.textio.all;

library neorv32;
use neorv32.neorv32_package.all;

entity xbus_memory is
  generic (
    MEM_SIZE : natural := 4; -- memory size in bytes; min 4; should be a power of two
    MEM_LATE : natural := 1; -- number of latency cycles (min 1)
    MEM_FILE : string  := "" -- memory initialization file (plain HEX), no initialization if empty
  );
  port (
    clk_i      : in  std_ulogic;
    rstn_i     : in  std_ulogic;
    xbus_req_i : in  xbus_req_t;
    xbus_rsp_o : out xbus_rsp_t
  );
end xbus_memory;

architecture xbus_memory_rtl of xbus_memory is

  -- address width --
  constant addr_bits_c : natural := index_size_f(MEM_SIZE/4);

  -- memory type --
  type ram8bv_t is array (natural range <>) of bit_vector(7 downto 0); -- bit_vector type for optimized simulator storage

  -- initialize ram8bv_t array from ASCII HEX file (no VHDL08 required) --
  impure function init_mem8bv_from_hexfile_f(file_name : string; num_words : natural; byte_sel : natural) return ram8bv_t is
    file     hex_file   : text;
    variable hex_line_v : line;
    variable hex_char_v : character;
    variable tmp_v      : natural;
    variable word_v     : bit_vector(31 downto 0);
    variable mem_v      : ram8bv_t(0 to num_words-1);
    variable index_v    : natural;
  begin
    mem_v := (others => (others => '0'));
    if (file_name /= "") then
      if (byte_sel = 0) then -- only print once
        report "[xbus_memory] Initializing memory from file '" & file_name & "'" severity warning;
      end if;
      file_open(hex_file, file_name, read_mode);
      index_v := 0;
      while (endfile(hex_file) = false) and (index_v < num_words) loop -- not end of file / end of memory
        readline(hex_file, hex_line_v); -- read one line from file
        for i in 7 downto 0 loop -- get full 32-bit word in 'word_v'; no VHDL2008 required
          read(hex_line_v, hex_char_v);
          case hex_char_v is
            when '0'       => tmp_v := 0;
            when '1'       => tmp_v := 1;
            when '2'       => tmp_v := 2;
            when '3'       => tmp_v := 3;
            when '4'       => tmp_v := 4;
            when '5'       => tmp_v := 5;
            when '6'       => tmp_v := 6;
            when '7'       => tmp_v := 7;
            when '8'       => tmp_v := 8;
            when '9'       => tmp_v := 9;
            when 'a' | 'A' => tmp_v := 10;
            when 'b' | 'B' => tmp_v := 11;
            when 'c' | 'C' => tmp_v := 12;
            when 'd' | 'D' => tmp_v := 13;
            when 'e' | 'E' => tmp_v := 14;
            when 'f' | 'F' => tmp_v := 15;
            when others    => tmp_v := 0;
          end case;
          word_v(i*4+3 downto i*4+0) := to_bitvector(std_ulogic_vector((to_unsigned(tmp_v, 4))));
        end loop;
        mem_v(index_v) := word_v(byte_sel*8+7 downto byte_sel*8+0); -- extract desired byte
        index_v := index_v + 1;
      end loop;
    end if;
    return mem_v;
  end function init_mem8bv_from_hexfile_f;

  -- memory access --
  signal addr  : unsigned(addr_bits_c-1 downto 0);
  signal rdata : std_ulogic_vector(31 downto 0);
  signal ack   : std_ulogic;

  -- latency generator --
  type late_data_t is array (MEM_LATE downto 0) of std_ulogic_vector(31 downto 0);
  signal late_data : late_data_t;
  signal late_ack  : std_ulogic_vector(MEM_LATE downto 0);
  signal dout      : std_ulogic_vector(31 downto 0);
  signal aout      : std_ulogic;

begin

  -- Pre-Initialized Memory (all-zero or HEX image if specified) ----------------------------
  -- -------------------------------------------------------------------------------------------
  memory_data: process(clk_i)
    -- [NOTE] The memory is split into four sub-memories using _variables_ of
    -- type 'bit_vector' to minimize the simulator's RAM footprint.
    variable mem8bv_b0_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 0);
    variable mem8bv_b1_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 1);
    variable mem8bv_b2_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 2);
    variable mem8bv_b3_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 3);
  begin
    if rising_edge(clk_i) then
      if (xbus_req_i.cyc = '1') and (xbus_req_i.stb = '1') then
        if (xbus_req_i.we = '1') then
          if (xbus_req_i.sel(0) = '1') then
            mem8bv_b0_v(to_integer(addr)) := to_bitvector(xbus_req_i.data(07 downto 00));
          end if;
          if (xbus_req_i.sel(1) = '1') then
            mem8bv_b1_v(to_integer(addr)) := to_bitvector(xbus_req_i.data(15 downto 08));
          end if;
          if (xbus_req_i.sel(2) = '1') then
            mem8bv_b2_v(to_integer(addr)) := to_bitvector(xbus_req_i.data(23 downto 16));
          end if;
          if (xbus_req_i.sel(3) = '1') then
            mem8bv_b3_v(to_integer(addr)) := to_bitvector(xbus_req_i.data(31 downto 24));
          end if;
        else
          if (xbus_req_i.sel(0) = '1') then
            rdata(07 downto 00) <= to_stdulogicvector(mem8bv_b0_v(to_integer(addr)));
          end if;
          if (xbus_req_i.sel(1) = '1') then
            rdata(15 downto 08) <= to_stdulogicvector(mem8bv_b1_v(to_integer(addr)));
          end if;
          if (xbus_req_i.sel(2) = '1') then
            rdata(23 downto 16) <= to_stdulogicvector(mem8bv_b2_v(to_integer(addr)));
          end if;
          if (xbus_req_i.sel(3) = '1') then
            rdata(31 downto 24) <= to_stdulogicvector(mem8bv_b3_v(to_integer(addr)));
          end if;
        end if;
      end if;
    end if;
  end process memory_data;

  addr <= unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2));

  memory_ack: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      ack <= '0';
    elsif rising_edge(clk_i) then
      ack <= xbus_req_i.cyc and xbus_req_i.stb;
    end if;
  end process memory_ack;


  -- Latency Generator ----------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  latency_gen: process(rstn_i, clk_i)
  begin
    if (rstn_i = '0') then
      late_data <= (others => (others => '0'));
      late_ack  <= (others => '0');
    elsif rising_edge(clk_i) then
      late_data(0) <= rdata;
      late_ack(0)  <= ack;
      for i in 0 to MEM_LATE-1 loop
        late_data(i+1) <= late_data(i);
        late_ack(i+1)  <= late_ack(i);
      end loop;
    end if;
  end process latency_gen;

  -- delay select --
  dout <= rdata when (MEM_LATE = 1) else late_data(MEM_LATE-1);
  aout <= ack   when (MEM_LATE = 1) else late_ack(MEM_LATE-1);

  -- bus response --
  xbus_rsp_o.data <= dout when (aout = '1') else (others => '0'); -- output gate
  xbus_rsp_o.ack  <= aout;
  xbus_rsp_o.err  <= '0';


end xbus_memory_rtl;
