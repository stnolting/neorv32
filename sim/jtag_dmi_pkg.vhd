-- ================================================================================ --
-- NEORV32 RISC-V JTAG/DMI Testbench Utilities Package                              --
-- ================================================================================ --
-- The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              --
-- Copyright (c) NEORV32 contributors.                                              --
-- Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  --
-- Licensed under the BSD-3-Clause license, see LICENSE for details.                --
-- SPDX-License-Identifier: BSD-3-Clause                                            --
-- ================================================================================ --

library ieee;
use ieee.std_logic_1164.all;

package jtag_dmi_pkg is

  -- JTAG clock period --
  constant t_jtag_c : time := 80 ns;

  -- Reset TAP controller state machine
  procedure jtag_reset(
    signal core_tck : out std_ulogic;
    signal core_tms : out std_ulogic;
    signal core_tdi : out std_ulogic;
    signal core_tdo : in  std_ulogic
  );

  -- Write to DMI register
  procedure dmi_write(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant addr     : in  std_ulogic_vector(6 downto 0);
    constant data     : in  std_ulogic_vector(31 downto 0)
  );

  -- Read from DMI register
  procedure dmi_read(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant addr     : in  std_ulogic_vector(6 downto 0);
    variable data     : out std_ulogic_vector(31 downto 0)
  );

end package jtag_dmi_pkg;

package body jtag_dmi_pkg is

  -- Generate single JTAG clock cycle and update TX / sample RX data ------------------------
  -- -------------------------------------------------------------------------------------------
  procedure jtag_tck_cycle(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant tms      : in  std_ulogic;
    constant tdi      : in  std_ulogic;
    variable tdo      : out std_ulogic
  ) is
  begin
    core_tck <= '0';
    core_tms <= tms;
    core_tdi <= tdi;
    wait for t_jtag_c / 4;
    core_tck <= '1';
    tdo := core_tdo;
    wait for t_jtag_c / 2;
    core_tck <= '0';
    wait for t_jtag_c / 4;
  end procedure jtag_tck_cycle;

  -- Reset TAP controller state machine -----------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  procedure jtag_reset(
    signal core_tck : out std_ulogic;
    signal core_tms : out std_ulogic;
    signal core_tdi : out std_ulogic;
    signal core_tdo : in  std_ulogic
  ) is
    variable tdo_v : std_ulogic;
  begin
    for i in 0 to 8 loop
      jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v); -- go to RESET from any state
    end loop;
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- RUN_IDLE
  end procedure jtag_reset;

  -- Set TAP instruction register (IR) ------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  procedure jtag_shift_ir(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant ir       : in  std_ulogic_vector(4 downto 0)
  ) is
    variable tdo_v : std_ulogic;
  begin
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v); -- SELECT-DR-SCAN
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v); -- SELECT-IR-SCAN
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- CAPTURE-IR
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- SHIFT-IR
    for i in 0 to 3 loop
      jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', ir(i), tdo_v); -- shift first 4 bits
    end loop;
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', ir(4), tdo_v); -- last bit: EXIT1-IR
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v);   -- UPDATE-IR
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v);   -- RUN-TEST/IDLE
  end procedure jtag_shift_ir;

  -- Set TAP data register (DMI only) -------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  procedure jtag_shift_dr(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant din      : in  std_ulogic_vector(40 downto 0);
    variable dout     : out std_ulogic_vector(40 downto 0)
  ) is
    variable tdo_v : std_ulogic;
  begin
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v); -- SELECT-DR-SCAN
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- CAPTURE-DR
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- SHIFT-DR
    for i in 0 to 40 loop
      if (i = 40) then
        jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', din(i), tdo_v); -- EXIT1-DR
      else
        jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', din(i), tdo_v); -- SHIFT-DR
      end if;
      dout(i) := tdo_v;
    end loop;
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '1', '0', tdo_v); -- UPDATE-DR
    jtag_tck_cycle(core_tck, core_tms, core_tdi, core_tdo, '0', '0', tdo_v); -- RUN-TEST/IDLE
  end procedure jtag_shift_dr;

  -- Write to DMI register ------------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  procedure dmi_write(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant addr     : in  std_ulogic_vector(6 downto 0);
    constant data     : in  std_ulogic_vector(31 downto 0)
  ) is
    variable dmi_in_v, dmi_out_v : std_ulogic_vector(40 downto 0);
  begin
    jtag_shift_ir(core_tck, core_tms, core_tdi, core_tdo, "10001"); -- IR = DMI
    dmi_in_v := addr & data & "10"; -- op = write
    jtag_shift_dr(core_tck, core_tms, core_tdi, core_tdo, dmi_in_v, dmi_out_v); -- write to TAP.dmi
  end procedure dmi_write;

  -- Read from DMI register -----------------------------------------------------------------
  -- -------------------------------------------------------------------------------------------
  procedure dmi_read(
    signal   core_tck : out std_ulogic;
    signal   core_tms : out std_ulogic;
    signal   core_tdi : out std_ulogic;
    signal   core_tdo : in  std_ulogic;
    constant addr     : in  std_ulogic_vector(6 downto 0);
    variable data     : out std_ulogic_vector(31 downto 0)
  ) is
    variable dmi_in_v, dmi_out_v : std_ulogic_vector(40 downto 0);
  begin
    jtag_shift_ir(core_tck, core_tms, core_tdi, core_tdo, "10001"); -- IR = DMI
    dmi_in_v := addr & x"00000000" & "01"; -- op = read
    jtag_shift_dr(core_tck, core_tms, core_tdi, core_tdo, dmi_in_v, dmi_out_v); -- write to TAP.dmi
    dmi_in_v := (others => '0'); -- op = NOP
    jtag_shift_dr(core_tck, core_tms, core_tdi, core_tdo, dmi_in_v, dmi_out_v); -- read from TAP.dmi
    data := dmi_out_v(33 downto 2);
  end procedure dmi_read;

end package body jtag_dmi_pkg;
