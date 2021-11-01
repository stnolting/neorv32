--   __   __     __  __     __         __
--  /\ "-.\ \   /\ \/\ \   /\ \       /\ \
--  \ \ \-.  \  \ \ \_\ \  \ \ \____  \ \ \____
--   \ \_\\"\_\  \ \_____\  \ \_____\  \ \_____\
--    \/_/ \/_/   \/_____/   \/_____/   \/_____/
--   ______     ______       __     ______     ______     ______
--  /\  __ \   /\  == \     /\ \   /\  ___\   /\  ___\   /\__  _\
--  \ \ \/\ \  \ \  __<    _\_\ \  \ \  __\   \ \ \____  \/_/\ \/
--   \ \_____\  \ \_____\ /\_____\  \ \_____\  \ \_____\    \ \_\
--    \/_____/   \/_____/ \/_____/   \/_____/   \/_____/     \/_/
--
-- https://joshbassett.info
-- https://twitter.com/nullobject
-- https://github.com/nullobject
--
-- Copyright (c) 2020 Josh Bassett
-- Copyright (c) 2021 Lawrie Griffiths (Contributor)
--
-- Permission is hereby granted, free of charge, to any person obtaining a copy
-- of this software and associated documentation files (the "Software"), to deal
-- in the Software without restriction, including without limitation the rights
-- to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
-- copies of the Software, and to permit persons to whom the Software is
-- furnished to do so, subject to the following conditions:
--
-- The above copyright notice and this permission notice shall be included in all
-- copies or substantial portions of the Software.
--
-- THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
-- IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
-- FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
-- AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
-- LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
-- OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
-- SOFTWARE.

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;
use ieee.math_real.all;

-- This SDRAM controller provides a symmetric 32-bit synchronous read/write
-- interface for a 16Mx16-bit SDRAM chip (e.g. AS4C16M16SA-6TCN, IS42S16400F,
-- etc.).
entity sdram is
  generic (
    -- clock frequency (in MHz)
    --
    -- This value must be provided, as it is used to calculate the number of
    -- clock cycles required for the other timing values.
    clk_freq : natural := 100;

    -- 32-bit controller interface
    ADDR_WIDTH : natural := 23;
    DATA_WIDTH : natural := 32;

    -- SDRAM interface
    SDRAM_ADDR_WIDTH : natural := 13;
    SDRAM_DATA_WIDTH : natural := 16;
    SDRAM_COL_WIDTH  : natural := 9;
    SDRAM_ROW_WIDTH  : natural := 13;
    SDRAM_BANK_WIDTH : natural := 2;

    -- The delay in clock cycles, between the start of a read command and the
    -- availability of the output data.
    CAS_LATENCY : natural := 2; -- 2=below 133MHz, 3=above 133MHz

    -- The number of 16-bit words to be bursted during a read/write.
    BURST_LENGTH : integer := 2;

    -- timing values (in nanoseconds)
    --
    -- These values can be adjusted to match the exact timing of your SDRAM
    -- chip (refer to the datasheet).
    T_DESL : real :=  50000.0; -- startup delay
    T_MRD  : real :=     12.0; -- mode register cycle time
    T_RC   : real :=     60.0; -- row cycle time
    T_RCD  : real :=     18.0; -- RAS to CAS delay
    T_RP   : real :=     18.0; -- precharge to activate delay
    T_WR   : real :=     12.0; -- write recovery time
    T_REFI : real :=   7800.0  -- average refresh interval
  );
  port (
    -- reset
    reset : in std_logic := '0';

    -- clock
    clk : in std_logic;

    -- address bus
    addr : in unsigned(ADDR_WIDTH-1 downto 0);

    -- input data bus
    data : in std_logic_vector(DATA_WIDTH-1 downto 0);

    -- When the write enable signal is asserted, a write operation will be performed.
    we : in std_logic;

    -- When the request signal is asserted, an operation will be performed.
    req : in std_logic;

    -- The acknowledge signal is asserted by the SDRAM controller when
    -- a request has been accepted.
    ack : out std_logic;

    -- The valid signal is asserted when there is a valid word on the output
    -- data bus.
    valid : out std_logic;

    -- output data bus
    q : out std_logic_vector(DATA_WIDTH-1 downto 0);

    -- SDRAM interface (e.g. AS4C16M16SA-6TCN, IS42S16400F, etc.)
    sdram_a     : out unsigned(SDRAM_ADDR_WIDTH-1 downto 0);
    sdram_ba    : out unsigned(SDRAM_BANK_WIDTH-1 downto 0);
    sdram_dq    : inout std_logic_vector(SDRAM_DATA_WIDTH-1 downto 0);
    sdram_cke   : out std_logic;
    sdram_cs_n  : out std_logic;
    sdram_ras_n : out std_logic;
    sdram_cas_n : out std_logic;
    sdram_we_n  : out std_logic;
    sdram_dqml  : out std_logic;
    sdram_dqmh  : out std_logic
  );
end sdram;

architecture arch of sdram is
  function ilog2(n : natural) return natural is
  begin
    return natural(ceil(log2(real(n))));
  end ilog2;

  subtype command_t is std_logic_vector(3 downto 0);

  -- commands
  constant CMD_DESELECT     : command_t := "1---";
  constant CMD_LOAD_MODE    : command_t := "0000";
  constant CMD_AUTO_REFRESH : command_t := "0001";
  constant CMD_PRECHARGE    : command_t := "0010";
  constant CMD_ACTIVE       : command_t := "0011";
  constant CMD_WRITE        : command_t := "0100";
  constant CMD_READ         : command_t := "0101";
  constant CMD_STOP         : command_t := "0110";
  constant CMD_NOP          : command_t := "0111";

  -- the ordering of the accesses within a burst
  constant BURST_TYPE : std_logic := '0'; -- 0=sequential, 1=interleaved

  -- the write burst mode enables bursting for write operations
  constant WRITE_BURST_MODE : std_logic := '0'; -- 0=burst, 1=single

  -- the value written to the mode register to configure the memory
  constant MODE_REG : unsigned(SDRAM_ADDR_WIDTH-1 downto 0) := (
    "000" &
    WRITE_BURST_MODE &
    "00" &
    to_unsigned(CAS_LATENCY, 3) &
    BURST_TYPE &
    to_unsigned(ilog2(BURST_LENGTH), 3)
  );

  -- calculate the clock period (in nanoseconds)
  constant CLK_PERIOD : real := (1.0/Real(clk_freq))*1000.0;

  -- the number of clock cycles to wait before initialising the device
  constant INIT_WAIT : natural := natural(ceil(T_DESL/CLK_PERIOD));

  -- the number of clock cycles to wait while a LOAD MODE command is being
  -- executed
  constant LOAD_MODE_WAIT : natural := natural(ceil(T_MRD/CLK_PERIOD));

  -- the number of clock cycles to wait while an ACTIVE command is being
  -- executed
  constant ACTIVE_WAIT : natural := natural(ceil(T_RCD/CLK_PERIOD));

  -- the number of clock cycles to wait while a REFRESH command is being
  -- executed
  constant REFRESH_WAIT : natural := natural(ceil(T_RC/CLK_PERIOD));

  -- the number of clock cycles to wait while a PRECHARGE command is being
  -- executed
  constant PRECHARGE_WAIT : natural := natural(ceil(T_RP/CLK_PERIOD));

  -- the number of clock cycles to wait while a READ command is being executed
  constant READ_WAIT : natural := CAS_LATENCY+BURST_LENGTH;

  -- the number of clock cycles to wait while a WRITE command is being executed
  constant WRITE_WAIT : natural := BURST_LENGTH+natural(ceil((T_WR+T_RP)/CLK_PERIOD));

  -- the number of clock cycles before the memory controller needs to refresh
  -- the SDRAM
  constant REFRESH_INTERVAL : natural := natural(floor(T_REFI/CLK_PERIOD))-10;

  type state_t is (INIT, MODE, IDLE, ACTIVE, READ, WRITE, REFRESH);

  -- state signals
  signal state, next_state : state_t;

  -- command signals
  signal cmd, next_cmd : command_t := CMD_NOP;

  -- control signals
  signal start          : std_logic;
  signal load_mode_done : std_logic;
  signal active_done    : std_logic;
  signal refresh_done   : std_logic;
  signal first_word     : std_logic;
  signal read_done      : std_logic;
  signal write_done     : std_logic;
  signal should_refresh : std_logic;

  -- counters
  signal wait_counter    : natural range 0 to 16383;
  signal refresh_counter : natural range 0 to 1023;

  -- registers
  signal addr_reg : unsigned(SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH+SDRAM_BANK_WIDTH-1 downto 0);
  signal data_reg : std_logic_vector(DATA_WIDTH-1 downto 0);
  signal we_reg   : std_logic;
  signal q_reg    : std_logic_vector(DATA_WIDTH-1 downto 0);

  -- aliases to decode the address register
  alias col  : unsigned(SDRAM_COL_WIDTH-1 downto 0) is addr_reg(SDRAM_COL_WIDTH-1 downto 0);
  alias row  : unsigned(SDRAM_ROW_WIDTH-1 downto 0) is addr_reg(SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH-1 downto SDRAM_COL_WIDTH);
  alias bank : unsigned(SDRAM_BANK_WIDTH-1 downto 0) is addr_reg(SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH+SDRAM_BANK_WIDTH-1 downto SDRAM_COL_WIDTH+SDRAM_ROW_WIDTH);
begin
  -- state machine
  fsm : process (state, wait_counter, req, we_reg, load_mode_done, active_done, refresh_done, read_done, write_done, should_refresh)
  begin
    next_state <= state;

    -- default to a NOP command
    next_cmd <= CMD_NOP;

    case state is
      -- execute the initialisation sequence
      when INIT =>
        if wait_counter = 0 then
          next_cmd <= CMD_DESELECT;
        elsif wait_counter = INIT_WAIT-1 then
          next_cmd <= CMD_PRECHARGE;
        elsif wait_counter = INIT_WAIT+PRECHARGE_WAIT-1 then
          next_cmd <= CMD_AUTO_REFRESH;
        elsif wait_counter = INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT-1 then
          next_cmd <= CMD_AUTO_REFRESH;
        elsif wait_counter = INIT_WAIT+PRECHARGE_WAIT+REFRESH_WAIT+REFRESH_WAIT-1 then
          next_state <= MODE;
          next_cmd   <= CMD_LOAD_MODE;
        end if;

      -- load the mode register
      when MODE =>
        if load_mode_done = '1' then
          next_state <= IDLE;
        end if;

      -- wait for a read/write request
      when IDLE =>
        if should_refresh = '1' then
          next_state <= REFRESH;
          next_cmd   <= CMD_AUTO_REFRESH;
        elsif req = '1' then
          next_state <= ACTIVE;
          next_cmd   <= CMD_ACTIVE;
        end if;

      -- activate the row
      when ACTIVE =>
        if active_done = '1' then
          if we_reg = '1' then
            next_state <= WRITE;
            next_cmd   <= CMD_WRITE;
          else
            next_state <= READ;
            next_cmd   <= CMD_READ;
          end if;
        end if;

      -- execute a read command
      when READ =>
        if read_done = '1' then
          if should_refresh = '1' then
            next_state <= REFRESH;
            next_cmd   <= CMD_AUTO_REFRESH;
          elsif req = '1' then
            next_state <= ACTIVE;
            next_cmd   <= CMD_ACTIVE;
          else
            next_state <= IDLE;
          end if;
        end if;

      -- execute a write command
      when WRITE =>
        if write_done = '1' then
          if should_refresh = '1' then
            next_state <= REFRESH;
            next_cmd   <= CMD_AUTO_REFRESH;
          elsif req = '1' then
            next_state <= ACTIVE;
            next_cmd   <= CMD_ACTIVE;
          else
            next_state <= IDLE;
          end if;
        end if;

      -- execute an auto refresh
      when REFRESH =>
        if refresh_done = '1' then
          if req = '1' then
            next_state <= ACTIVE;
            next_cmd   <= CMD_ACTIVE;
          else
            next_state <= IDLE;
          end if;
        end if;
    end case;
  end process;

  -- latch the next state
  latch_next_state : process (clk, reset)
  begin
    if reset = '1' then
      state <= INIT;
      cmd   <= CMD_NOP;
    elsif rising_edge(clk) then
      state <= next_state;
      cmd   <= next_cmd;
    end if;
  end process;

  -- the wait counter is used to hold the current state for a number of clock
  -- cycles
  update_wait_counter : process (clk, reset)
  begin
    if reset = '1' then
      wait_counter <= 0;
    elsif rising_edge(clk) then
      if state /= next_state then -- state changing
        wait_counter <= 0;
      else
        wait_counter <= wait_counter + 1;
      end if;
    end if;
  end process;

  -- the refresh counter is used to periodically trigger a refresh operation
  update_refresh_counter : process (clk, reset)
  begin
    if reset = '1' then
      refresh_counter <= 0;
    elsif rising_edge(clk) then
      if state = REFRESH and wait_counter = 0 then
        refresh_counter <= 0;
      else
        refresh_counter <= refresh_counter + 1;
      end if;
    end if;
  end process;

  -- latch the rquest
  latch_request : process (clk)
  begin
    if rising_edge(clk) then
      if start = '1' then
        -- we need to multiply the address by two, because we are converting
        -- from a 32-bit controller address to a 16-bit SDRAM address
        addr_reg <= shift_left(resize(addr, addr_reg'length), 1);
        data_reg <= data;
        we_reg   <= we;
      end if;
    end if;
  end process;

  -- latch the output data as it's bursted from the SDRAM
  latch_sdram_data : process (clk)
  begin
    if rising_edge(clk) then
      valid <= '0';

      if state = READ then
        if first_word = '1' then
          q_reg(31 downto 16) <= sdram_dq;
        elsif read_done = '1' then
          q_reg(15 downto 0) <= sdram_dq;
          valid <= '1';
        end if;
      end if;
    end if;
  end process;

  -- set wait signals
  load_mode_done <= '1' when wait_counter = LOAD_MODE_WAIT-1 else '0';
  active_done    <= '1' when wait_counter = ACTIVE_WAIT-1    else '0';
  refresh_done   <= '1' when wait_counter = REFRESH_WAIT-1   else '0';
  first_word     <= '1' when wait_counter = CAS_LATENCY      else '0';
  read_done      <= '1' when wait_counter = READ_WAIT-1      else '0';
  write_done     <= '1' when wait_counter = WRITE_WAIT-1     else '0';

  -- the SDRAM should be refreshed when the refresh interval has elapsed
  should_refresh <= '1' when refresh_counter >= REFRESH_INTERVAL-1 else '0';

  -- a new request is only allowed at the end of the IDLE, READ, WRITE, and
  -- REFRESH states
  start <= '1' when (state = IDLE) or
                    (state = READ and read_done = '1') or
                    (state = WRITE and write_done = '1') or
                    (state = REFRESH and refresh_done = '1') else '0';

  -- assert the acknowledge signal at the beginning of the ACTIVE state
  ack <= '1' when state = ACTIVE and wait_counter = 0 else '0';

  -- set output data
  q <= q_reg;

  -- deassert the clock enable at the beginning of the INIT state
  sdram_cke <= '0' when state = INIT and wait_counter = 0 else '1';

  -- set SDRAM control signals
  (sdram_cs_n, sdram_ras_n, sdram_cas_n, sdram_we_n) <= cmd;

  -- set SDRAM bank
  with state select
    sdram_ba <=
      bank            when ACTIVE,
      bank            when READ,
      bank            when WRITE,
      (others => '0') when others;

  -- set SDRAM address
  with state select
    sdram_a <=
      "0010000000000" when INIT,
      MODE_REG        when MODE,
      row             when ACTIVE,
      "0010" & col    when READ,   -- auto precharge
      "0010" & col    when WRITE,  -- auto precharge
      (others => '0') when others;

  -- decode the next 16-bit word from the write buffer
  sdram_dq <=
      (others => 'Z') when state /= WRITE else           -- tristate when not a WRITE
      data_reg(31 downto 16) when wait_counter = 0 else  -- first cycle of WRITE writes most significant 16 bits
      data_reg(15 downto 0);                             -- second cycle writes least significant 16 bits
  -- set SDRAM data mask
  sdram_dqmh <= '0';
  sdram_dqml <= '0';
end architecture arch;
