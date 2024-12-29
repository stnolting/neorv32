/* ================================================================================ */
/* NEORV32 CPU - RISC-V GCC Linker Script                                           */
/* -------------------------------------------------------------------------------- */
/* "Park loop" code for execution-based on-chip debugger'S (OCD) code ROM.          */
/* -------------------------------------------------------------------------------- */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */

OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(_ocd_start)

MEMORY
{
  debug_rom (rx) : ORIGIN = 0xFFFFFE00, LENGTH = 128
}

SECTIONS
{
  .text :
  {
    KEEP(*(.text.ocd));
  } > debug_rom
}
