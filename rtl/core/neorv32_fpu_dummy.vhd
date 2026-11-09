-- args: --std=08 --ieee=synopsys

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity neorv32_fpu_dummy is
  port (
    rstn_i    : in  std_ulogic;
    clk_i     : in  std_ulogic;
    rs1       : in  std_ulogic_vector(63 downto 0);
    rs2       : in  std_ulogic_vector(63 downto 0);
    rs3       : in  std_ulogic_vector(63 downto 0);
    fmadd     : in  std_ulogic;
    fmsub     : in  std_ulogic;
    fnmadd    : in  std_ulogic;
    fnmsub    : in  std_ulogic;
    fadd      : in  std_ulogic;
    fsub      : in  std_ulogic;
    fmul      : in  std_ulogic;
    fdiv      : in  std_ulogic;
    fsqrt     : in  std_ulogic;
    fsgnj     : in  std_ulogic;
    fcmp      : in  std_ulogic;
    fmax      : in  std_ulogic;
    fclass    : in  std_ulogic;
    fmv_i2f   : in  std_ulogic;
    fmv_f2i   : in  std_ulogic;
    fcvt_f2f  : in  std_ulogic;
    fcvt_i2f  : in  std_ulogic;
    fcvt_f2i  : in  std_ulogic;
    fcvt_op   : in  std_ulogic_vector(1 downto 0);
    fmt       : in  std_ulogic_vector(1 downto 0);
    rm        : in  std_ulogic_vector(2 downto 0);
    res       : out std_ulogic_vector(63 downto 0);
    flags     : out std_ulogic_vector(4 downto 0);
    ready     : out std_ulogic;
    clear     : in  std_ulogic
  );
end entity neorv32_fpu_dummy;

architecture neorv32_fpu_dummy_rtl of neorv32_fpu_dummy is
begin

  res   <= (others => '0');
  flags <= (others => '0');
  ready <= '0';

end architecture neorv32_fpu_dummy_rtl;
