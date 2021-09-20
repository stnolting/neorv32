from typing import List, Dict
from os import environ


class Design:
    def __init__(self, name: str, vhdl: List[str], verilog: List[str] = None):
        self.Name = name
        self.VHDL = vhdl
        self.Verilog = verilog


class Filesets:
    def __init__(
        self, designs: List[Design], imem: Dict[str, str], dmem: Dict[str, str]
    ):
        self.Designs = designs
        self.InstructionMemory = imem
        self.DataMemory = dmem


class Project:
    def __init__(self, root):
        # These paths need to be relative to 'setups/osflow/', which is the location of the make entrypoint ('common.mk')
        self.RTLDir = "../../rtl"
        self.BoardsTops = "board_tops"

        # Get the list of supported boards from the names of the '*.mk' files in 'setups/osflow/boards'
        self.Boards = [
            str(item.stem)
            for item in (root / "setups" / "osflow" / "boards").glob("*.mk")
            if item.stem != "index"
        ]

        # Get the revision of the Boards expected to need it
        self.Board_Revisions = {
            "Fomu": environ.get("FOMU_REV", "pvt"),
            "UPduino": environ.get("UPduino_REV", "v3"),
            "OrangeCrab": environ.get("OrangeCrab_REV", "r02-25F"),
        }

        self.Filesets = Filesets(
            # HDL sources of each design
            designs=[
                Design(
                    "Minimal",
                    vhdl=[
                        "{}/processor_templates/neorv32_ProcessorTop_Minimal*.vhd".format(self.RTLDir)
                    ],
                ),
                Design(
                    "MinimalBoot",
                    vhdl=[
                        "{}/processor_templates/neorv32_ProcessorTop_MinimalBoot.vhd".format(self.RTLDir)
                    ],
                ),
                Design(
                    "UP5KDemo",
                    vhdl=[
                        "{}/processor_templates/neorv32_ProcessorTop_UP5KDemo.vhd".format(self.RTLDir)
                    ],
                ),
                Design(
                    "MixedLanguage",
                    vhdl=[
                        "{}/processor_templates/neorv32_ProcessorTop_Minimal*.vhd".format(self.RTLDir)
                    ],
                    verilog=[
                        "devices/ice40/sb_ice40_components.v",
                        "{}/neorv32_Fomu_MixedLanguage_ClkGen.v".format(self.BoardsTops),
                    ],
                ),
            ],
            # Variants of the HDL sources for IMEM and DMEM
            imem={
                "default": "{}/core/mem/neorv32_imem.default.vhd".format(self.RTLDir),
                "ice40up_spram": "devices/ice40/neorv32_imem.ice40up_spram.vhd",
            },
            dmem={
                "default": "{}/core/mem/neorv32_dmem.default.vhd".format(self.RTLDir),
                "ice40up_spram": "devices/ice40/neorv32_dmem.ice40up_spram.vhd",
            },
        )

    def GetMemorySources(self, board: str, design: str) -> List[str]:
        """
        Define which sources are used for Instruction and Data memories, depending on the target Board and/or Design
        """
        imem = "ice40up_spram"
        dmem = "ice40up_spram"

        if board == "Fomu":
            imem = "default" if design == "Minimal" else "ice40up_spram"

        elif board in ["OrangeCrab", "AlhambraII", "ULX3S"]:
            imem = "default"
            dmem = "default"

        return [self.Filesets.InstructionMemory[imem], self.Filesets.DataMemory[dmem]]
