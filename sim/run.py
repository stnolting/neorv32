#!/usr/bin/env python3

from pathlib import Path
from vunit import VUnit

ROOT = Path(__file__).parent

PRJ = VUnit.from_argv()

PRJ.add_library("neorv32").add_source_files([
    ROOT / "*.vhd",
    ROOT / "../rtl/**/*.vhd"
])

PRJ.set_sim_option("disable_ieee_warnings", True)

PRJ.main()
