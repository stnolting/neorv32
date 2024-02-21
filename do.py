#!/usr/bin/env python3

# doit

from sys import executable, argv as sys_argv, exit as sys_exit
from os import environ
from pathlib import Path

from doit.action import CmdAction
from doit.cmd_base import ModuleTaskLoader
from doit.doit_cmd import DoitMain

DOIT_CONFIG = {"verbosity": 2, "action_string_formatting": "both"}

ROOT = Path(__file__).parent


def task_SoftwareFrameworkTests():
    return {
        "actions": [
            # Check toolchain
            "make -C sw/example/processor_check check",
            # Generate executables for all example projects
            "make -C sw/example clean_all exe",
            # Compile and install bootloader
            "make -C sw/bootloader clean_all info bootloader",
        ],
        "doc": "Build all sw/example/*; install bootloader",
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
