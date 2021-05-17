from pathlib import Path
from vunit import VUnit

root = Path(__file__).parent
prj = VUnit.from_argv()
prj.add_com()
prj.add_verification_components()

neorv32 = prj.add_library("neorv32")
neorv32.add_source_files(root / "*.vhd")
neorv32.add_source_files(root / "../rtl/**/*.vhd")

prj.set_sim_option("disable_ieee_warnings", True)

prj.main()
