#!/usr/bin/env python3

import json
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
PRJ.add_vhdl_builtins()
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

def _gen_vhdl_ls(vu):
    """
    Generate the vhdl_ls.toml file required by VHDL-LS language server.
    """
    # Repo root
    parent = Path(__file__).parent.parent

    proj = vu._project
    libs = proj.get_libraries()

    with open(parent / 'vhdl_ls.toml', "w") as f:
        for lib in libs:
            f.write(f"[libraries.{lib.name}]\n")
            files = [str(file).replace('\\', '/') for file in lib._source_files
                # Conflicts with *.default.vhd
                if not any(exclude in file for exclude in ('neorv32_imem.simple.vhd', 'neorv32_imem.legacy.vhd', 'neorv32_dmem.legacy.vhd'))
            ]
            f.write(f"files = {json.dumps(files, indent=4)}\n")

_gen_vhdl_ls(PRJ)

PRJ.main()
