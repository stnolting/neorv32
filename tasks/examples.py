from os import environ
from pathlib import Path

from .project import Project


ROOT = Path(__file__).parent.parent

PRJ = Project(ROOT)


def Run(
    board: str,
    design: str,
    top: str,
    id: str,
    board_srcs: list,
    design_srcs: list,
    verilog_srcs: list,
    mem_srcs: list,
    posargs: list,
) -> list:
    """
    Create command to call the make entrypoint 'setups/osflow/common.mk' for executing 'posargs' targets.
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


def Example(board: str, design: str, posargs: str) -> str:
    """
    Call the 'Run' function to get the make command of a given example (Board and Design) for executing 'posargs' targets.
    """

    if board not in PRJ.Boards:
        raise Exception("Unknown board {}".format(board))

    DesignSources = next((item for item in PRJ.Filesets.Designs if item.Name == design), None)

    if DesignSources == None:
        raise Exception("Unknown design {}".format(design))

    boardtop = "neorv32_{}_BoardTop_{}".format(board, design)

    if not (ROOT / "setups/osflow" / PRJ.BoardsTops / "{}.vhd".format(boardtop)).exists():
        raise Exception("BoardTop file {} does not exist!".format(boardtop))

    # FIXME It should be possible to pass the command as a list, i.e., without converting it to a single string
    return " ".join(
        Run(
            board=board,
            design=design,
            top=boardtop,
            id=design,
            board_srcs=["{}/{}.vhd".format(PRJ.BoardsTops, boardtop)],
            design_srcs=DesignSources.VHDL,
            verilog_srcs=DesignSources.Verilog,
            mem_srcs=PRJ.GetMemorySources(board, design),
            posargs=posargs,
        )
    )


# TODO Add a task to be executed after Example for moving the bitstream from setups/osflow/*.bit to somewhere else
# (maybe setups/examples or setups/examples/out)
#
#   bitstream = "neorv32_{}{}_{}.bit".format(
#       board,
#       "_{}".format(PRJ.Board_Revisions[board]) if board in PRJ.Board_Revisions else "",
#       design
#   )
