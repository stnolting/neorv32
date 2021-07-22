#!/usr/bin/env python3

from pathlib import Path
from vunit import VUnit

ROOT = Path(__file__).resolve().parent

VU = VUnit.from_argv()
VU.add_verification_components()

RTLSRC = ROOT.parent.parent.parent / "rtl"

NEORV32 = VU.add_library("neorv32")
NEORV32.add_source_files([
    RTLSRC / "templates" / "processor" / "*.vhd",
    RTLSRC / "core" / "*.vhd",
    #RTLSRC / "core" / "imem" / "neorv32_imem.vhd",
    RTLSRC / "core" / "mem" / "neorv32_dmem.vhd",
    RTLSRC / "core" / "mem" / "neorv32_imem.alt.vhd",
])

VU.add_library("test").add_source_files(ROOT / "*.vhd")

VU.set_sim_option("disable_ieee_warnings", True)
#VU.set_sim_option("ghdl.sim_flags", ["--stop-time=20ms"])

VU.main()
