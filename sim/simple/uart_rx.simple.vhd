library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use std.textio.all;

entity uart_rx_simple is
  generic (
    name : string;
    uart_baud_val_c : real);

  port (
    clk : in std_ulogic;
    uart_txd : in std_ulogic
    );
end entity;

architecture a of uart_rx_simple is
  signal uart_rx_sync : std_ulogic_vector(04 downto 0) := (others => '1');
  signal uart_rx_busy : std_ulogic := '0';
  signal uart_rx_sreg : std_ulogic_vector(08 downto 0) := (others => '0');
  signal uart_rx_baud_cnt : real;
  signal uart_rx_bitcnt : natural;

  file file_uart_tx_out : text open write_mode is "neorv32.testbench_" & name & ".out";

begin
  uart_rx_console : process(clk)
    variable i : integer;
    variable l : line;
  begin
    -- "UART" --
    if rising_edge(clk) then
      -- synchronizer --
      uart_rx_sync <= uart_rx_sync(3 downto 0) & uart_txd;
      -- arbiter --
      if (uart_rx_busy = '0') then  -- idle
        uart_rx_busy <= '0';
        uart_rx_baud_cnt <= round(0.5 * uart_baud_val_c);
        uart_rx_bitcnt <= 9;
        if (uart_rx_sync(4 downto 1) = "1100") then  -- start bit? (falling edge)
          uart_rx_busy <= '1';
        end if;
      else
        if (uart_rx_baud_cnt <= 0.0) then
          if (uart_rx_bitcnt = 1) then
            uart_rx_baud_cnt <= round(0.5 * uart_baud_val_c);
          else
            uart_rx_baud_cnt <= round(uart_baud_val_c);
          end if;
          if (uart_rx_bitcnt = 0) then
            uart_rx_busy <= '0';  -- done
            i := to_integer(unsigned(uart_rx_sreg(8 downto 1)));

            if (i < 32) or (i > 32+95) then  -- printable char?
              report name & ".tx: (" & integer'image(i) & ")";  -- print code
            else
              report name & ".tx: " & character'val(i);  -- print ASCII
            end if;

            if (i = 10) then  -- Linux line break
              writeline(file_uart_tx_out, l);
            elsif (i /= 13) then  -- Remove additional carriage return
              write(l, character'val(i));
            end if;
          else
            uart_rx_sreg <= uart_rx_sync(4) & uart_rx_sreg(8 downto 1);
            uart_rx_bitcnt <= uart_rx_bitcnt - 1;
          end if;
        else
          uart_rx_baud_cnt <= uart_rx_baud_cnt - 1.0;
        end if;
      end if;
    end if;
  end process uart_rx_console;
end architecture;
