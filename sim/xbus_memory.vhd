-- ================================================================================ --
-- NEORV32 - Simple XBUS / Wishbone Memory (meant for simulation only)              --
-- -------------------------------------------------------------------------------- --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  --
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
  type ram8bv_t is array (natural range <>) of bit_vector(7 downto 0); -- bit_vector type for optimized system storage

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
      file_open(hex_file, file_name, read_mode);
      index_v := 0;
      while (endfile(hex_file) = false) and (index_v < num_words) loop -- not end of file / end of memory
        readline(hex_file, hex_line_v); -- read one line from file
        for i in 7 downto 0 loop -- get full 32-bit word in 'word_v'; no VHDL2008 required
          read(hex_line_v, hex_char_v);
          if (hex_char_v >= '0') and (hex_char_v <= '9') then
            tmp_v := 0 + (character'pos(hex_char_v) - character'pos('0'));
          elsif (hex_char_v >= 'a') and (hex_char_v <= 'f') then
            tmp_v := 10 + (character'pos(hex_char_v) - character'pos('a'));
          elsif (hex_char_v >= 'A') and (hex_char_v <= 'F') then
            tmp_v := 10 + (character'pos(hex_char_v) - character'pos('A'));
          else -- invalid character
            tmp_v := 0;
          end if;
          word_v(i*4+3 downto i*4+0) := to_bitvector(std_ulogic_vector((to_unsigned(tmp_v, 4))));
        end loop;
        mem_v(index_v) := word_v(byte_sel*8+7 downto byte_sel*8+0); -- extract desired byte
        index_v := index_v + 1;
      end loop;
    end if;
    return mem_v;
  end function init_mem8bv_from_hexfile_f;

  -- memory access --
  signal rdata : std_ulogic_vector(31 downto 0);
  signal ack   : std_ulogic;

  -- latency generator --
  type late_data_t is array (MEM_LATE downto 0) of std_ulogic_vector(31 downto 0);
  signal late_data : late_data_t;
  signal late_ack  : std_ulogic_vector(MEM_LATE downto 0);

begin

  -- Pre-Initialized Memory (all-zero or HEX image if specified) ----------------------------
  -- -------------------------------------------------------------------------------------------
  memory_core: process(clk_i)
    variable mem8bv_b0_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 0);
    variable mem8bv_b1_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 1);
    variable mem8bv_b2_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 2);
    variable mem8bv_b3_v : ram8bv_t(0 to (MEM_SIZE/4)-1) := init_mem8bv_from_hexfile_f(MEM_FILE, MEM_SIZE/4, 3);
  begin
    if rising_edge(clk_i) then
      ack <= '0';
      if (xbus_req_i.cyc = '1') and (xbus_req_i.stb = '1') then
        ack <= '1';
        if (xbus_req_i.we = '1') then
          if (xbus_req_i.sel(0) = '1') then
            mem8bv_b0_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))) := to_bitvector(xbus_req_i.data(07 downto 00));
          end if;
          if (xbus_req_i.sel(1) = '1') then
            mem8bv_b1_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))) := to_bitvector(xbus_req_i.data(15 downto 08));
          end if;
          if (xbus_req_i.sel(2) = '1') then
            mem8bv_b2_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))) := to_bitvector(xbus_req_i.data(23 downto 16));
          end if;
          if (xbus_req_i.sel(3) = '1') then
            mem8bv_b3_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))) := to_bitvector(xbus_req_i.data(31 downto 24));
          end if;
        else
          rdata(07 downto 00) <= to_stdulogicvector(mem8bv_b0_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))));
          rdata(15 downto 08) <= to_stdulogicvector(mem8bv_b1_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))));
          rdata(23 downto 16) <= to_stdulogicvector(mem8bv_b2_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))));
          rdata(31 downto 24) <= to_stdulogicvector(mem8bv_b3_v(to_integer(unsigned(xbus_req_i.addr(addr_bits_c+1 downto 2)))));
        end if;
      end if;
    end if;
  end process memory_core;


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

  -- bus response --
  xbus_rsp_o.data <= rdata when (MEM_LATE = 1) else late_data(MEM_LATE-1);
  xbus_rsp_o.ack  <= ack   when (MEM_LATE = 1) else late_ack(MEM_LATE-1);
  xbus_rsp_o.err  <= '0';


end xbus_memory_rtl;
