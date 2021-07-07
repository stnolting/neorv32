# doit

from os import environ

from doit.action import CmdAction

from tasks.examples import Example, PRJ

BOARDS = PRJ.Boards

DOIT_CONFIG = {"verbosity": 2}


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
