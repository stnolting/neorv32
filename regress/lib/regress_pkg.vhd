-- NEORV32 regression - helpers shared by all generated testbenches.
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

package regress_pkg is

  type suv32_list_t is array (natural range <>) of std_ulogic_vector(31 downto 0);

  -- trace value: hexadecimal when clean, binary when it holds metavalues
  function vstr(v : std_ulogic) return string;
  function vstr(v : std_ulogic_vector) return string;

  -- deterministic xorshift generator, the same on every machine
  subtype rng_t is unsigned(31 downto 0);
  function rng_seed_f(seed : natural) return rng_t;
  procedure rng_next(variable s : inout rng_t; variable v : out std_ulogic_vector);

end package;

package body regress_pkg is

  function vstr(v : std_ulogic) return string is
  begin
    return std_ulogic'image(v)(2 to 2);
  end function;

  function vstr(v : std_ulogic_vector) return string is
    variable clean : boolean := true;
  begin
    for i in v'range loop
      if (v(i) /= '0') and (v(i) /= '1') then
        clean := false;
      end if;
    end loop;
    if clean then
      return to_hstring(v);
    end if;
    return "b" & to_string(v);
  end function;

  function rng_seed_f(seed : natural) return rng_t is
    variable s : rng_t := to_unsigned(seed, 32) xor x"9E3779B9";
  begin
    if s = 0 then
      s := x"00000001";
    end if;
    return s;
  end function;

  procedure rng_next(variable s : inout rng_t; variable v : out std_ulogic_vector) is
    variable o : std_ulogic_vector(v'length - 1 downto 0);
    variable n : natural := 0;
  begin
    while n < v'length loop
      s := s xor shift_left(s, 13);
      s := s xor shift_right(s, 17);
      s := s xor shift_left(s, 5);
      for i in 0 to 31 loop
        if n < v'length then
          o(n) := s(i);
          n := n + 1;
        end if;
      end loop;
    end loop;
    v := o;
  end procedure;

end package body;
