# doit

from sys import executable
from os import environ
from pathlib import Path

from doit.action import CmdAction

from tasks.examples import Example, GenerateExamplesJobMatrix, PRJ

BOARDS = PRJ.Boards

DOIT_CONFIG = {"verbosity": 2, "action_string_formatting": "both"}

ROOT = Path(__file__).parent

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


def task_sim():
    simdir = ROOT / "sim"
    yield {
        "name": "Simple",
        "actions": [str(simdir / "simple/ghdl.sh")],
        "doc": "Run simple testbench with GHDL",
        "uptodate": [False],
    }
    yield {
        "name": "VUnit",
        # FIXME: It should we possible to use '--' for separating the args to be passed raw to the action, instead of
        # requiring a param and wrapping all the args in a single string
        "actions": ["{} {} {{args}}".format(executable, str(simdir / "run.py"))],
        "doc": "Run VUnit testbench",
        "uptodate": [False],
        "params": [
            {
                "name": "args",
                "short": "a",
                "long": "args",
                "default": "--ci-mode -v",
                "help": "Arguments to pass to the VUnit script",
            }
        ],
    }


def task_GenerateExamplesJobMatrix():
    return {
        "actions": [GenerateExamplesJobMatrix],
        "doc": "Generate JSON of the examples, and print it as 'set-output' (for CI)",
    }


def task_SetupRISCVGCC():
    return {
        "actions": [
            "mkdir riscv",
            "curl -fsSL https://github.com/stnolting/riscv-gcc-prebuilt/releases/download/rv32i-2.0.0/riscv32-unknown-elf.gcc-10.2.0.rv32i.ilp32.newlib.tar.gz | tar -xzf - -C riscv",
            "ls -al riscv",
        ],
        "doc": "Download and extract stnolting/riscv-gcc-prebuilt to subdir 'riscv'",
    }


def task_BuildAndInstallCheckSoftware():
    return {
        "actions": [
            " ".join(
                [
                    "make -C sw/example/processor_check",
                    "clean_all",
                    "USER_FLAGS+=-DRUN_CHECK",
                    "USER_FLAGS+=-DUART0_SIM_MODE",
                    "USER_FLAGS+=-DSUPPRESS_OPTIONAL_UART_PRINT",
                    "MARCH=-march=rv32imac",
                    "info",
                    "all",
                ]
            )
        ],
        "doc": "Build and install Processor Check software",
    }


def task_BuildAndInstallSoftwareFrameworkTests():
    return {
        "actions": [
            # Check toolchain
            "make -C sw/example/processor_check check",
            # Generate executables for all example projects
            "make -C sw/example clean_all exe",
            # Compile and install bootloader
            "make -C sw/bootloader clean_all info bootloader",
            # Compile and install test application
            # Redirect UART0 TX to text.io simulation output via <UART0_SIM_MODE> user flag
            "echo 'Compiling and installing CPU (/Processor) test application'",
            "make -C sw/example/processor_check clean_all USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DUART1_SIM_MODE MARCH=-march=rv32imac info all",
        ],
        "doc": "Build all sw/example/*; install bootloader and processor check",
    }
