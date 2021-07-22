library ieee;
context ieee.ieee_std_context;

library neorv32;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.vc_context;

entity neorv32_ProcessorTop_MinimalBoot_tb is
  generic (runner_cfg : string);
end entity;

architecture tb of neorv32_ProcessorTop_MinimalBoot_tb is

  -- configuration --
  constant f_clock_c : natural := 18000000; -- PLL output clock frequency in Hz

  constant t_clock_c : time := (1 sec) / f_clock_c;

  signal clk_gen : std_ulogic := '0';
  signal rst_gen : std_ulogic := '0';

  signal con_pwm : std_ulogic_vector(2 downto 0);
  signal con_gpio_o : std_ulogic_vector(3 downto 0);

  constant uart_bfm : uart_slave_t := new_uart_slave(initial_baud_rate => 19200);
  constant uart_stream : stream_slave_t := as_stream(uart_bfm);

  signal uart_txd_o : std_ulogic;

begin

  -- Clock/Reset Generator ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  clk_gen <= not clk_gen after (t_clock_c/2);
  rst_gen <= '0', '1' after 60*(t_clock_c/2);

  main : process
  begin
    test_runner_setup(runner, runner_cfg);
    wait for 25 ms;
    test_runner_cleanup(runner);
  end process;

  uut: entity neorv32.neorv32_ProcessorTop_MinimalBoot
  generic map (
    CLOCK_FREQUENCY => f_clock_c  -- clock frequency of clk_i in Hz
  )
  port map (
    -- Global control --
    clk_i      => clk_gen,
    rstn_i     => rst_gen,

    -- GPIO --
    gpio_o     => con_gpio_o,

    -- primary UART --
    uart_txd_o => uart_txd_o, -- UART0 send data
    uart_rxd_i => '0',        -- UART0 receive data
    uart_rts_o => open,       -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart_cts_i => '0',        -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- PWM (available if IO_PWM_EN = true) --
    pwm_o      => con_pwm
  );

  uart_master_bfm : entity vunit_lib.uart_slave
  generic map (
    uart => uart_bfm
  )
  port map (
    rx => uart_txd_o
  );

  print_uart: process
    variable data : std_logic_vector(7 downto 0);
  begin
    pop_stream(net, uart_stream, data);
    info(to_string(character'val(to_integer(unsigned(data)))));
  end process;

  print_gpio: process
  begin
    wait for 1 ms;
    info("gpio_o(0): " & to_string(con_gpio_o(0)));
  end process;

end architecture;
