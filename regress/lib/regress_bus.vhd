-- NEORV32 regression - bus host procedure and memory model for unit tests.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

package regress_bus_pkg is

  -- one complete bus access, ends one cycle after the response
  procedure bus_xfer(
    signal clk    : in  std_ulogic;
    signal req    : out bus_req_t;
    signal rsp    : in  bus_rsp_t;
    a, d          : in  std_ulogic_vector(31 downto 0);
    rw            : in  std_ulogic;
    ben           : in  std_ulogic_vector(3 downto 0) := "1111";
    meta          : in  std_ulogic_vector(4 downto 0) := "00000";
    amo           : in  std_ulogic := '0';
    amoop         : in  std_ulogic_vector(3 downto 0) := "0000";
    burst         : in  std_ulogic := '0';
    lock          : in  std_ulogic := '0';
    limit         : in  natural := 5000
  );

end package;

package body regress_bus_pkg is

  procedure bus_xfer(
    signal clk    : in  std_ulogic;
    signal req    : out bus_req_t;
    signal rsp    : in  bus_rsp_t;
    a, d          : in  std_ulogic_vector(31 downto 0);
    rw            : in  std_ulogic;
    ben           : in  std_ulogic_vector(3 downto 0) := "1111";
    meta          : in  std_ulogic_vector(4 downto 0) := "00000";
    amo           : in  std_ulogic := '0';
    amoop         : in  std_ulogic_vector(3 downto 0) := "0000";
    burst         : in  std_ulogic := '0';
    lock          : in  std_ulogic := '0';
    limit         : in  natural := 5000
  ) is
    variable n : natural := 0;
  begin
    req <= (meta => meta, addr => a, data => d, ben => ben, stb => '1', rw => rw,
            amo => amo, amoop => amoop, burst => burst, lock => lock);
    wait until rising_edge(clk);
    req.stb <= '0';
    loop
      wait until falling_edge(clk);
      exit when rsp.ack = '1';
      n := n + 1;
      exit when n >= limit;
    end loop;
    wait until rising_edge(clk);
    req.amo <= '0';
    req.lock <= '0';
  end procedure;

end package body;


library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

-- 1 KiB word memory with byte enables. LATENCY cycles until the response. Addresses with
-- upper nibble E answer with an error, D never answer.
entity regress_bus_mem is
  generic (
    LATENCY : natural := 1
  );
  port (
    clk_i : in  std_ulogic;
    req_i : in  bus_req_t;
    rsp_o : out bus_rsp_t
  );
end entity;

architecture sim of regress_bus_mem is
  type mem_t is array (0 to 255) of std_ulogic_vector(31 downto 0);
begin

  process
    variable mem : mem_t := (others => x"DEADBEEF");
    variable idx : natural;
    variable r   : bus_rsp_t;
    variable pend, skip : boolean := false;
  begin
    wait until rising_edge(clk_i);
    rsp_o <= rsp_terminate_c;
    if req_i.stb = '1' then
      idx := to_integer(unsigned(req_i.addr(9 downto 2)));
      r := rsp_terminate_c;
      r.ack := '1';
      skip := false;
      if req_i.addr(31 downto 28) = x"E" then
        r.err := '1';
      elsif req_i.addr(31 downto 28) = x"D" then
        skip := true;
      elsif req_i.rw = '1' then
        for i in 0 to 3 loop
          if req_i.ben(i) = '1' then
            mem(idx)(8*i+7 downto 8*i) := req_i.data(8*i+7 downto 8*i);
          end if;
        end loop;
      else
        r.data := mem(idx);
      end if;
      if not skip then
        for i in 2 to LATENCY loop
          wait until rising_edge(clk_i);
          rsp_o <= rsp_terminate_c;
        end loop;
        rsp_o <= r;
      end if;
    end if;
  end process;

end architecture;
