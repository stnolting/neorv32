/* ================================================================================ */
/* NEORV32 CPU - RISC-V GCC Linker Script                                           */
/* -------------------------------------------------------------------------------- */
/* For the execution-based on-chip debugger (OCD) code memory ROM: "park loop" code */
/* (build-in firmware)                                                              */
/* -------------------------------------------------------------------------------- */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */

OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(__start)

MEMORY
{
  debug_mem (rx) : ORIGIN = 0xFFFFFF00, LENGTH = 128
}

SECTIONS
{
  .text :
  {
    KEEP(*(.text.ocd));
  } > debug_mem
}
