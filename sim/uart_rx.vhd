library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

use std.textio.all;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

use work.uart_rx_pkg.all;

entity uart_rx is
  generic (
    actor : actor_t;
    logger : logger_t;
    uart_baud_val_c : real);

  port (
    clk : in std_ulogic;
    uart_txd : in std_ulogic
    );
end entity;

architecture a of uart_rx is
  signal uart_rx_sync : std_ulogic_vector(04 downto 0) := (others => '1');
  signal uart_rx_busy : std_ulogic := '0';
  signal uart_rx_sreg : std_ulogic_vector(08 downto 0) := (others => '0');
  signal uart_rx_baud_cnt : real;
  signal uart_rx_bitcnt : natural;

  file file_uart_tx_out : text open write_mode is "neorv32.testbench_" & get_name(logger) & ".out";
  constant checker : checker_t := new_checker(logger);
  constant character_queue : queue_t := new_queue;

begin
  control : process
    variable msg : msg_t;
    variable msg_type : msg_type_t;

    procedure put_characters_in_queue(s : string) is
    begin
      for idx in s'range loop
        push(character_queue, s(idx));
      end loop;
    end procedure put_characters_in_queue;
  begin
    receive(net, actor, msg);
    msg_type := message_type(msg);

    if msg_type = check_uart_msg then
      put_characters_in_queue(pop(msg));
    else
      unexpected_msg_type(msg_type);
    end if;
  end process;

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

            if is_empty(character_queue) then
              check_failed(checker, "Extra characters received");
            end if;
            check_equal(checker, character'val(i), pop(character_queue));

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
