/* ================================================================================ */
/* NEORV32 CPU - OCD Park loop code linker script (for DM code ROM)                 */
/* -------------------------------------------------------------------------------- */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */

OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(ocd_start)

MEMORY
{
  dm_code_rom (rx) : ORIGIN = 0xFFFFFF00, LENGTH = 64
}

SECTIONS
{
  .text :
  {
    KEEP(*(.text.ocd));
  } > dm_code_rom
}
