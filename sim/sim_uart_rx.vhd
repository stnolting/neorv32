-- ================================================================================ --
-- NEORV32 - Simulation UART Receiver (print to simulator console and log to file)  --
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
use ieee.math_real.all;

-- pragma translate_off
-- RTL_SYNTHESIS OFF
use std.textio.all;
-- RTL_SYNTHESIS ON
-- pragma translate_on

entity sim_uart_rx is
  generic (
    NAME : string; -- receiver name
    FCLK : real; -- clock speed of clk_i in Hz
    BAUD : real -- baud rate
  );
  port (
    clk : in std_ulogic; -- global clock
    rxd : in std_ulogic -- serial UART RX data
  );
end entity sim_uart_rx;

architecture sim_uart_rx_rtl of sim_uart_rx is

  signal sync : std_ulogic_vector(4 downto 0) := (others => '1');
  signal busy : std_ulogic := '0';
  signal sreg : std_ulogic_vector(8 downto 0) := (others => '0');
  signal baudcnt : real;
  signal bitcnt : integer;
  constant baud_val_c : real := FCLK / BAUD;

begin

-- pragma translate_off
-- RTL_SYNTHESIS OFF
  sim_receiver: process(clk)
    file file_out   : text open write_mode is NAME & ".log";
    variable char_v : integer;
    variable line_v : line;
  begin
    if rising_edge(clk) then
      -- synchronizer --
      sync <= sync(3 downto 0) & rxd;
      -- start trigger --
      if (busy = '0') then -- idle
        busy    <= '0';
        baudcnt <= round(0.5 * baud_val_c);
        bitcnt  <= 9;
        if (sync(4 downto 1) = "1100") then -- start bit (falling edge)
          busy <= '1';
        end if;
      -- receiving --
      elsif (baudcnt <= 0.0) then
        if (bitcnt = 1) then
          baudcnt <= round(0.5 * baud_val_c);
        else
          baudcnt <= round(baud_val_c);
        end if;
        if (bitcnt = 0) then
          busy <= '0'; -- done
          char_v := to_integer(unsigned(sreg(8 downto 1)));
          if (char_v < 32) or (char_v > 32+95) then -- non-printable character?
            report NAME & ": (" & integer'image(char_v) & ")";
          else
            report NAME & ": " & character'val(char_v);
          end if;
          if (char_v = 10) then -- LF line break
            writeline(file_out, line_v);
          elsif (char_v /= 13) then -- no additional CR
            write(line_v, character'val(char_v));
          end if;
        else
          sreg   <= sync(4) & sreg(8 downto 1);
          bitcnt <= bitcnt - 1;
        end if;
      else
        baudcnt <= baudcnt - 1.0;
      end if;
    end if;
  end process sim_receiver;
-- RTL_SYNTHESIS ON
-- pragma translate_on

end architecture sim_uart_rx_rtl;
