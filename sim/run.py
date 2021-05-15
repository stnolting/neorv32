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
prj = VUnit.from_args(args=args)

root = Path(__file__).parent

neorv32 = prj.add_library("neorv32")
neorv32.add_source_files(root / "*.vhd")
neorv32.add_source_files(root / "../rtl/**/*.vhd")

tb = neorv32.test_bench("neorv32_tb")
end_of_text = chr(3)
tb.set_generic("ci_mode", args.ci_mode)

prj.set_sim_option("disable_ieee_warnings", True)

prj.main()
