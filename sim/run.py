#!/usr/bin/env python3

from pathlib import Path
from vunit import VUnit, VUnitCLI

cli = VUnitCLI()
cli.parser.add_argument(
    "--ci-mode",
    action="store_true",
    default=False,
    help="Enable special settings used by the CI",
)
args = cli.parse_args()

PRJ = VUnit.from_args(args=args)
PRJ.add_com()
PRJ.add_verification_components()
PRJ.add_osvvm()

ROOT = Path(__file__).parent

NEORV32 = PRJ.add_library("neorv32")
NEORV32.add_source_files([
    ROOT / "*.vhd",
    ROOT / ".." / "rtl" / "**" / "*.vhd",
    # In VUnit <=v4.5.0, the glob search is not recursive,
    # hence subdir 'mem' is not picked by the previous pattern
    ROOT / ".." / "rtl" / "core" / "mem" / "*.vhd"
])

NEORV32.test_bench("neorv32_tb").set_generic("ci_mode", args.ci_mode)

PRJ.set_sim_option("disable_ieee_warnings", True)
PRJ.set_sim_option("ghdl.sim_flags", ["--max-stack-alloc=256"])

PRJ.main()
