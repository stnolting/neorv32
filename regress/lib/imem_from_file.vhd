-- NEORV32 regression - IMEM image read from "image.hex" in the run directory.
-- This replaces the generated rtl/core/neorv32_imem_image.vhd, which is plain data, so that a
-- single compiled simulator can run any program. The file holds one 32-bit hex word per line.
library ieee;
use ieee.std_logic_1164.all;
use std.textio.all;

package neorv32_imem_image is

  type rom_t is array (0 to 65535) of std_ulogic_vector(31 downto 0);
  impure function image_load_f return rom_t;
  impure function image_size_f return natural;
  constant image_data_c : rom_t;
  constant image_size_c : natural;

end package;

package body neorv32_imem_image is

  impure function image_load_f return rom_t is
    file f : text;
    variable st : file_open_status;
    variable l : line;
    variable w : std_ulogic_vector(31 downto 0);
    variable rom : rom_t := (others => x"00000013"); -- nop
    variable n : natural := 0;
  begin
    file_open(st, f, "image.hex", read_mode);
    if st = open_ok then
      while not endfile(f) loop
        readline(f, l);
        if l'length >= 8 then
          hread(l, w);
          rom(n) := w;
          n := n + 1;
        end if;
      end loop;
      file_close(f);
    end if;
    return rom;
  end function;

  impure function image_size_f return natural is
    file f : text;
    variable st : file_open_status;
    variable l : line;
    variable n : natural := 0;
  begin
    file_open(st, f, "image.hex", read_mode);
    if st = open_ok then
      while not endfile(f) loop
        readline(f, l);
        if l'length >= 8 then
          n := n + 4;
        end if;
      end loop;
      file_close(f);
    end if;
    if n = 0 then
      n := 256; -- no program: 64 words of nop
    end if;
    return n;
  end function;

  constant image_data_c : rom_t := image_load_f;
  constant image_size_c : natural := image_size_f;

end package body;
