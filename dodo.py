# doit

from sys import executable
from os import environ
from pathlib import Path

from doit.action import CmdAction

DOIT_CONFIG = {"verbosity": 2, "action_string_formatting": "both"}

ROOT = Path(__file__).parent


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
