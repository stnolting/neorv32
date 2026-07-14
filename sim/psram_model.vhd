-- ================================================================================ --
-- NEORV32 - Simple SPI/QPI PSRAM Model                                             --
-- -------------------------------------------------------------------------------- --
-- [NOTE] This module was developed using AI tools. Use with care.                  --
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

entity psram_model is
  generic (
    MEM_BYTES : natural := 8*1024 -- memory size in bytes, has to be a power of 2, accesses wrap around
  );
  port (
    sck  : in    std_logic;                   -- clock
    cs_n : in    std_logic;                   -- chip select, low-active
    sio  : inout std_logic_vector(3 downto 0) -- data interface (sio[0]=SDI, sio[1]=SDO)
  );
end entity;

architecture psram_model_sim of psram_model is

  type mem_t   is array (0 to MEM_BYTES-1) of std_logic_vector(7 downto 0);
  type phase_t is (P_CMD, P_ADDR, P_DUMMY, P_READ, P_WRITE, P_ID, P_DONE);
  type id_t    is array (0 to 7) of std_logic_vector(7 downto 0);

  constant ADDR_MASK : unsigned(23 downto 0) := to_unsigned(MEM_BYTES-1, 24);
  constant ID : id_t := (x"01", x"23", x"45", x"67", x"89", x"ab", x"cd", x"ef");

  -- states and stuff --
  signal mem      : mem_t   := (others => (others => '0'));
  signal phase    : phase_t := P_CMD;
  signal qpi      : boolean := false;
  signal cmd      : std_logic_vector(7 downto 0) := (others => '0');
  signal din      : std_logic_vector(7 downto 0) := (others => '0');
  signal outsr    : std_logic_vector(7 downto 0) := (others => '0');
  signal addr     : unsigned(23 downto 0) := (others => '0');
  signal bitcnt   : integer := 0;
  signal ibits    : integer := 0;
  signal obits    : integer := 8;
  signal dcnt     : integer := 0;
  signal dummy_c  : integer := 0;
  signal addr_w   : integer := 1;
  signal data_w   : integer := 1;
  signal idx      : integer := 0;
  signal is_read  : boolean := false;
  signal is_write : boolean := false;
  signal is_id    : boolean := false;

begin

  psram_core: process (sck, cs_n)

    -- Decode command (sets operating mode / next phase) --------------------------------------
    -- -------------------------------------------------------------------------------------------
    procedure decode_f(constant c : std_logic_vector(7 downto 0)) is
    begin
      addr     <= (others => '0');
      bitcnt   <= 0;
      is_read  <= false;
      is_write <= false;
      is_id    <= false;
      dummy_c  <= 0;
      if qpi then -- interface mode QPI
        addr_w <= 4;
        data_w <= 4;
      else -- interface mode SPI
        addr_w <= 1;
        data_w <= 1;
      end if;
      case c is -- access command
        when x"03" => is_read  <= true;                                          phase <= P_ADDR; -- read
        when x"0B" => is_read  <= true; dummy_c <= 8;                            phase <= P_ADDR; -- fast read
        when x"EB" => is_read  <= true; addr_w <= 4; data_w <= 4; dummy_c <= 6;  phase <= P_ADDR; -- fast read quad
        when x"02" => is_write <= true;                                          phase <= P_ADDR; -- write
        when x"38" => is_write <= true; addr_w <= 4; data_w <= 4;                phase <= P_ADDR; -- quad write
        when x"9F" => is_id    <= true;                                          phase <= P_ADDR; -- read id
        when x"35" => qpi <= true;                                               phase <= P_DONE; -- enter qpi
        when x"F5" => qpi <= false;                                              phase <= P_DONE; -- exit qpi
        when others =>                                                           phase <= P_DONE; -- incl. reset 0x66/0x99
      end case;
    end procedure;

    -- Start access once the full address is received (am is already masked) ------------------
    -- -------------------------------------------------------------------------------------------
    procedure start_access_f(constant am : unsigned(23 downto 0)) is
    begin
      addr   <= am;
      bitcnt <= 0;
      if is_id then
        outsr <= ID(0);
        idx   <= 1;
        obits <= 8;
        phase <= P_ID;
      elsif (dummy_c > 0) then
        dcnt  <= 0;
        phase <= P_DUMMY;
      elsif is_read then
        outsr <= mem(to_integer(am));
        obits <= 8;
        phase <= P_READ;
      else
        ibits <= 0;
        phase <= P_WRITE;
      end if;
    end procedure;

    -- Load next output byte (read burst / ID rollover) ---------------------------------------
    -- -------------------------------------------------------------------------------------------
    procedure load_next_f is
    begin
      if phase = P_ID then
        outsr <= ID(idx);
        idx   <= (idx + 1) mod ID'length;
      else
        outsr <= mem(to_integer((addr + 1) and ADDR_MASK));
        addr  <= (addr + 1) and ADDR_MASK; -- wrap around
      end if;
      obits <= 8;
    end procedure;

  begin
    if cs_n = '1' then
      phase  <= P_CMD;
      bitcnt <= 0;
      cmd    <= (others => '0');
      sio    <= (others => 'Z');

    -- Input: sample on rising edge (clock mode 0) --------------------------------------------
    -- -------------------------------------------------------------------------------------------
    elsif rising_edge(sck) then
      case phase is

        when P_CMD =>
          if qpi then
            if bitcnt = 4 then -- second nibble completes the byte
              decode_f(cmd(3 downto 0) & sio);
            else
              cmd    <= cmd(3 downto 0) & sio;
              bitcnt <= bitcnt + 4;
            end if;
          else
            if (bitcnt = 7) then -- eighth bit completes the byte
              decode_f(cmd(6 downto 0) & sio(0));
            else
              cmd    <= cmd(6 downto 0) & sio(0);
              bitcnt <= bitcnt + 1;
            end if;
          end if;

        when P_ADDR =>
          if (addr_w = 4) then
            if (bitcnt = 20) then -- 6th nibble => 24 bits complete
              start_access_f((addr(19 downto 0) & unsigned(sio)) and ADDR_MASK);
            else
              addr   <= addr(19 downto 0) & unsigned(sio);
              bitcnt <= bitcnt + 4;
            end if;
          else
            if (bitcnt = 23) then -- 24th bit complete
              start_access_f((addr(22 downto 0) & sio(0)) and ADDR_MASK);
            else
              addr   <= addr(22 downto 0) & sio(0);
              bitcnt <= bitcnt + 1;
            end if;
          end if;

        when P_DUMMY =>
          if (dcnt = dummy_c - 1) then
            outsr <= mem(to_integer(addr));
            obits <= 8;
            phase <= P_READ;
          else
            dcnt <= dcnt + 1;
          end if;

        when P_WRITE =>
          if (data_w = 4) then
            if (ibits = 4) then
              mem(to_integer(addr)) <= din(3 downto 0) & sio;
              addr  <= (addr + 1) and ADDR_MASK; -- wrap around
              ibits <= 0;
            else
              din   <= din(3 downto 0) & sio;
              ibits <= ibits + 4;
            end if;
          else
            if (ibits = 7) then
              mem(to_integer(addr)) <= din(6 downto 0) & sio(0);
              addr  <= (addr + 1) and ADDR_MASK; -- wrap around
              ibits <= 0;
            else
              din   <= din(6 downto 0) & sio(0);
              ibits <= ibits + 1;
            end if;
          end if;

        when others => null; -- P_READ / P_ID / P_DONE: ignore input

      end case;

    -- Output: drive on falling edge (clock mode 0) -------------------------------------------
    -- -------------------------------------------------------------------------------------------
    elsif falling_edge(sck) then
      if (phase = P_READ) or (phase = P_ID) then
        if (data_w = 4) then
          sio <= outsr(7 downto 4);
          if obits = 4 then
            load_next_f;
          else
            outsr <= outsr(3 downto 0) & "0000";
            obits <= obits - 4;
          end if;
        else
          sio(3) <= 'Z';
          sio(2) <= 'Z';
          sio(1) <= outsr(7); -- drive only SO (IO1)
          sio(0) <= 'Z';
          if (obits = 1) then
            load_next_f;
          else
            outsr <= outsr(6 downto 0) & '0';
            obits <= obits - 1;
          end if;
        end if;
      else
        sio <= (others => 'Z');
      end if;
    end if;
  end process psram_core;

end architecture psram_model_sim;
