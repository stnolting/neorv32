# doit

from os import environ
from pathlib import Path

from doit.action import CmdAction


DOIT_CONFIG = {"verbosity": 2}

ROOT = Path(__file__).parent

# These paths need to be relative to 'setups/osflow/', which is the location of the make entrypoint ('common.mk')
TEMPLATES = "../../rtl/templates"
EXAMPLES = "../examples"

# Get the list of supported boards from the names of the '*.mk' files in 'setups/osflow/boards'
BOARDS = [str(item.stem) for item in (ROOT / "setups" / "osflow" / "boards").glob("*.mk") if item.stem != "index"]

# Get the revision of the Boards expected to need it
BOARD_REVS = {
    "Fomu": environ.get("FOMU_REV", "pvt"),
    "UPduino": environ.get("UPduino_REV", "v3"),
    "OrangeCrab": environ.get("OrangeCrab_REV", "r02-25F"),
}

# HDL sources of each design
DESIGNS = {
    "Minimal": {"vhdl": ["{}/processor/neorv32_ProcessorTop_Minimal*.vhd".format(TEMPLATES)]},
    "MinimalBoot": {"vhdl": ["{}/processor/neorv32_ProcessorTop_MinimalBoot.vhd".format(TEMPLATES)]},
    "UP5KDemo": {"vhdl": ["{}/processor/neorv32_ProcessorTop_UP5KDemo.vhd".format(TEMPLATES)]},
    "MixedLanguage": {
        "vhdl": ["{}/processor/neorv32_ProcessorTop_Minimal*.vhd".format(TEMPLATES)],
        "verilog": ["devices/ice40/sb_ice40_components.v", "{}/neorv32_Fomu_MixedLanguage_ClkGen.v".format(EXAMPLES)],
    },
}

# Variants of the HDL sources for IMEM and DMEM
IMEM_SRCS = {
    "default": "../../rtl/core/neorv32_imem.vhd",
    "ice40up_spram": "devices/ice40/neorv32_imem.ice40up_spram.vhd",
}
DMEM_SRCS = {
    "default": "../../rtl/core/neorv32_dmem.vhd",
    "ice40up_spram": "devices/ice40/neorv32_dmem.ice40up_spram.vhd",
}


def Run(board, design, top, id, board_srcs, design_srcs, verilog_srcs, mem_srcs, posargs):
    """
    Create command to call the make entrypoint 'setups/osflow/common.mk' to execute 'posargs' targets.
    """
    cmd = [
        "make",
        "-C",
        "setups/osflow",
        "-f",
        "common.mk",
        "BOARD='{}'".format(board),
        "DESIGN='{}'".format(design),
        "BOARD_SRC='{}'".format(" ".join(board_srcs)),
        "TOP='{}'".format(top),
        "ID='{}'".format(id),
        "DESIGN_SRC='{}'".format(" ".join(design_srcs)),
        "NEORV32_MEM_SRC='{}'".format(" ".join(mem_srcs)),
    ]

    if verilog_srcs is not None:
        cmd.append("NEORV32_VERILOG_SRC='{}'".format(" ".join(verilog_srcs)))

    cmd += posargs if posargs != [] else ["clean", "bit"]

    return cmd


def Example(board, design, posargs):
    """
    Call the 'Run' function to get the make command for a given example (Board and Design).
    """

    if board not in BOARDS:
        raise Exception("Unknown board {}".format(board))

    if design not in DESIGNS:
        raise Exception("Unknown design {}".format(design))

    design_srcs = DESIGNS[design]

    boardtop = "neorv32_{}_BoardTop_{}".format(board, design)

    if not (ROOT / "setups" / "examples" / "{}.vhd".format(boardtop)).exists():
        raise Exception("BoardTop file {} does not exist!".format(boardtop))

    # FIXME It should be possible to pass the command as a list, i.e., without converting it to a single string
    return " ".join(Run(
        board=board,
        design=design,
        top=boardtop,
        id=design,
        board_srcs=["{}/{}.vhd".format(EXAMPLES, boardtop)],
        design_srcs=design_srcs["vhdl"],
        verilog_srcs=design_srcs["verilog"] if "verilog" in design_srcs else None,
        # Here, we define which sources are used for IMEM and DMEM, depending on the target Board and/or Design
        mem_srcs=(
            [
                IMEM_SRCS["default"] if design == "Minimal" else IMEM_SRCS["ice40up_spram"], DMEM_SRCS["ice40up_spram"]
            ]
            if board == "Fomu"
            else [IMEM_SRCS["default"], DMEM_SRCS["default"]]
            if board == "OrangeCrab"
            else [IMEM_SRCS["ice40up_spram"], DMEM_SRCS["ice40up_spram"]]
        ),
        posargs=posargs,
    ))


# TODO Add a task to be executed after Example for moving the bitstream from setups/osflow/*.bit to somewhere else
# (maybe setups/examples or setups/examples/out)
#
#   bitstream = "neorv32_{}{}_{}.bit".format(
#       board,
#       "_{}".format(BOARD_REVS[board]) if board in BOARD_REVS else "",
#       design
#   )


#
# CLI (doit) entrypoints
#


# > doit list --all
#
# For instance, build example Minimal for Fomu and load it
# > doit Example -b Fomu -d Minimal clean load
#
# Or, alternatively, using sub-tasks (https://pydoit.org/tasks.html#sub-tasks):
# > doit ex:Fomu -d Minimal clean load
#
def task_Example():
    yield {
        "basename": "ex",
        "name": None,
        "doc": "Build an example design for all the supported boards",
    }
    for board in BOARDS:
        yield {
            "basename": "ex",
            "name": board,
            "actions": [CmdAction((Example, [], {"board": board}))],
            "doc": "Build an example design for board {}".format(board),
            "uptodate": [False],
            "pos_arg": "posargs",
            "params": [
                {
                    "name": "design",
                    "short": "d",
                    "long": "design",
                    "type": str,
                    "default": environ.get("DESIGN", "MinimalBoot"),
                    "help": "Name of the design",
                },
            ],
        }
    yield {
        "basename": "Example",
        "actions": [CmdAction(Example)],
        "doc": "Build an example design for a board",
        "uptodate": [False],
        "pos_arg": "posargs",
        "params": [
            {
                "name": "board",
                "short": "b",
                "long": "board",
                "type": str,
                "choices": ((board, "") for board in BOARDS),
                "default": environ.get("BOARD", "Fomu"),
                "help": "Name of the board",
            },
            {
                "name": "design",
                "short": "d",
                "long": "design",
                "type": str,
                "default": environ.get("DESIGN", "MinimalBoot"),
                "help": "Name of the design",
            },
        ],
    }
