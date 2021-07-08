#!/usr/bin/env python3

# doit

from sys import executable, argv as sys_argv, exit as sys_exit
from os import environ
from pathlib import Path

from doit.action import CmdAction
from doit.cmd_base import ModuleTaskLoader
from doit.doit_cmd import DoitMain

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


def task_GenerateExamplesJobMatrix():
    return {
        "actions": [GenerateExamplesJobMatrix],
        "doc": "Generate JSON of the examples, and print it as 'set-output' (for CI)",
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


def task_RunRISCVArchitectureTests():
    return {
        "actions": [CmdAction(
            "./run_riscv_arch_test.sh {suite}",
            cwd=ROOT / "sim"
        )],
        "doc": "Run RISC-V Architecture Tests",
        "params": [
            {
                "name": "suite",
                "short": "s",
                "long": "suite",
                "default": "M",
                "choices": ((item, "") for item in [
                    "I",
                    "C",
                    "M",
                    "privilege",
                    "Zifencei",
                    "rv32e_C",
                    "rv32e_E",
                    "rv32e_M"
                ]),
                "help": "Test suite to be executed",
            }
        ],
    }


def task_Documentation():
    return {
        "actions": ["make -C docs {posargs}"],
        "doc": "Run a target in subdir 'doc'",
        "uptodate": [False],
        "pos_arg": "posargs",
    }


def task_DeployToGitHubPages():
    cwd = str(ROOT / "public")
    return {
        "actions": [
            CmdAction(cmd, cwd=cwd)
            for cmd in [
                "git init",
                "cp ../.git/config ./.git/config",
                "touch .nojekyll",
                "git add .",
                'git config --local user.email "push@gha"',
                'git config --local user.name "GHA"',
                "git commit -am '{posargs}'",
                "git push -u origin +HEAD:gh-pages",
            ]
        ],
        "doc": "Create a clean branch in subdir 'public' and push to branch 'gh-pages'",
        "pos_arg": "posargs",
    }


if __name__ == '__main__':
    sys_exit(DoitMain(ModuleTaskLoader(globals())).run(sys_argv[1:]))
