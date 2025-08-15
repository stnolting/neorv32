/* ================================================================================ */
/* NEORV32 - RISC-V GCC Linker Script                                               */
/* -------------------------------------------------------------------------------- */
/* The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              */
/* Copyright (c) NEORV32 contributors.                                              */
/* Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  */
/* Licensed under the BSD-3-Clause license, see LICENSE for details.                */
/* SPDX-License-Identifier: BSD-3-Clause                                            */
/* ================================================================================ */

OUTPUT_FORMAT("elf32-littleriscv")
OUTPUT_ARCH(riscv)
ENTRY(_start)

/* ************************************************************************************************* */
/* NEORV32 memory layout configuration                                                               */
/* If the "__neorv32_*" symbols are not **explicitly defined** the default configurations are used.  */
/* [NOTE] section sizes have to be a multiple of 4 bytes; base addresses have to be 32-bit-aligned.  */
/* ************************************************************************************************* */

/* Default HEAP size (= 0; no heap by default) */
__neorv32_heap_size = DEFINED(__neorv32_heap_size) ? __neorv32_heap_size : 0;

/* Default rom/ram (IMEM/DMEM) sizes */
__neorv32_rom_size = DEFINED(__neorv32_rom_size) ? __neorv32_rom_size : 16k;
__neorv32_ram_size = DEFINED(__neorv32_ram_size) ? __neorv32_ram_size : 8K;

/* Default rom/ram (IMEM/DMEM) base addresses */
__neorv32_rom_base = DEFINED(__neorv32_rom_base) ? __neorv32_rom_base : 0x00000000;
__neorv32_ram_base = DEFINED(__neorv32_ram_base) ? __neorv32_ram_base : 0x80000000;

/* ************************************************************************************************* */
/* Main memory segments that are relevant for the executable.                                        */
/* ************************************************************************************************* */
MEMORY
{
  rom  (rx) : ORIGIN = __neorv32_rom_base, LENGTH = __neorv32_rom_size
  ram (rwx) : ORIGIN = __neorv32_ram_base, LENGTH = __neorv32_ram_size
}

/* ************************************************************************************************* */
/* Section ".text" - program code                                                                    */
/* ************************************************************************************************* */
SECTIONS
{
  .text : ALIGN(4)
  {
    PROVIDE(__text_start = .);

    /* keep start-up code crt0 right at the beginning of rom */
    KEEP(*(.text.crt0));

    *(.text .text* .text.* .gnu.linkonce.t.*)

    . = ALIGN(4);
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP (*(.preinit_array))
    KEEP (*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
    KEEP (*(.init_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .ctors))
    PROVIDE_HIDDEN(__init_array_end = .);

    KEEP (*(SORT_NONE(.init)))
    KEEP (*(SORT_NONE(.fini)))

    . = ALIGN(4);
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP (*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
    KEEP (*(.fini_array EXCLUDE_FILE (*crtbegin.o *crtbegin?.o *crtend.o *crtend?.o ) .dtors))
    PROVIDE_HIDDEN(__fini_array_end = .);

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__text_end = .);
  } > rom

/* ************************************************************************************************* */
/* Section ".rodata" - read-only constants and ".data" initialization                                */
/* ************************************************************************************************* */
  .rodata : ALIGN(4)
  {
    PROVIDE(__rodata_start = .);

    /* constant data like strings */
    *(.rodata .rodata* .rodata.* .gnu.linkonce.r.*)

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__rodata_end = .);
  } > rom

/* ************************************************************************************************* */
/* Section ".data" - pre-initialized variables                                                       */
/* crt0 will initialize this RAM section from the executable's ".data" section during boot-up        */
/* ************************************************************************************************* */
  .data : ALIGN(4)
  {
    PROVIDE(__data_start = .);
    __global_pointer = . + 0x800;

    *(.data .data.* .data* .gnu.linkonce.d.*)
    *(.srodata .srodata.*)
    *(.sdata .sdata.* .gnu.linkonce.s.*)
    *(.tdata .tdata.* .gnu.linkonce.td.*)

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__data_end = .);
  } > ram AT > rom

/* ************************************************************************************************* */
/* Section ".bss" - non-initialized variables; cleared by crt0 during boot-up                        */
/* ************************************************************************************************* */
  .bss (NOLOAD): ALIGN(4)
  {
    PROVIDE(__bss_start = .);

    *(.sbss .sbss* .sbss.* .gnu.linkonce.sb.*)
    *(.tbss .tbss.* .gnu.linkonce.tb.*) *(.tcommon)
    *(.scommon)
    *(.dynamic)
    *(.dynsbss)
    *(.dynbss)
    *(.bss .bss* .bss.* .gnu.linkonce.b.*)

    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);

    *(COMMON)
    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__bss_end = .);
  } > ram

/* ************************************************************************************************* */
/* Section ".heap" - dynamic memory (e.g. malloc)                                                    */
/* ************************************************************************************************* */
  .heap : ALIGN(4)
  {
    PROVIDE(__heap_start = .);

    . += __neorv32_heap_size;

    /* finish section on WORD boundary */
    . = ALIGN(4);
    PROVIDE(__heap_end = .);
  } > ram

/* ************************************************************************************************* */
/* Unused sections                                                                                   */
/* ************************************************************************************************* */
  .jcr               : { KEEP (*(.jcr)) }
  .got               : { *(.got.plt) *(.igot.plt) *(.got) *(.igot) } .interp : { *(.interp) }
  .note.gnu.build-id : { *(.note.gnu.build-id) }
  .hash              : { *(.hash) }
  .gnu.hash          : { *(.gnu.hash) }
  .dynsym            : { *(.dynsym) }
  .dynstr            : { *(.dynstr) }
  .gnu.version       : { *(.gnu.version) }
  .gnu.version_d     : { *(.gnu.version_d) }
  .gnu.version_r     : { *(.gnu.version_r) }
  .rela.init         : { *(.rela.init) }
  .rela.text         : { *(.rela.text .rela.text.* .rela.gnu.linkonce.t.*) }
  .rela.fini         : { *(.rela.fini) }
  .rela.rodata       : { *(.rela.rodata .rela.rodata.* .rela.gnu.linkonce.r.*) }
  .rela.data.rel.ro  : { *(.rela.data.rel.ro .rela.data.rel.ro.* .rela.gnu.linkonce.d.rel.ro.*) }
  .rela.data         : { *(.rela.data .rela.data.* .rela.gnu.linkonce.d.*) }
  .rela.tdata        : { *(.rela.tdata .rela.tdata.* .rela.gnu.linkonce.td.*) }
  .rela.tbss         : { *(.rela.tbss .rela.tbss.* .rela.gnu.linkonce.tb.*) }
  .rela.ctors        : { *(.rela.ctors) }
  .rela.dtors        : { *(.rela.dtors) }
  .rela.got          : { *(.rela.got) }
  .rela.sdata        : { *(.rela.sdata .rela.sdata.* .rela.gnu.linkonce.s.*) }
  .rela.sbss         : { *(.rela.sbss .rela.sbss.* .rela.gnu.linkonce.sb.*) }
  .rela.sdata2       : { *(.rela.sdata2 .rela.sdata2.* .rela.gnu.linkonce.s2.*) }
  .rela.sbss2        : { *(.rela.sbss2 .rela.sbss2.* .rela.gnu.linkonce.sb2.*) }
  .rela.bss          : { *(.rela.bss .rela.bss.* .rela.gnu.linkonce.b.*) }

/* ************************************************************************************************* */
/* Debug symbols                                                                                     */
/* ************************************************************************************************* */
  .stab            0 : { *(.stab) }
  .stabstr         0 : { *(.stabstr) }
  .stab.excl       0 : { *(.stab.excl) }
  .stab.exclstr    0 : { *(.stab.exclstr) }
  .stab.index      0 : { *(.stab.index) }
  .stab.indexstr   0 : { *(.stab.indexstr) }
  .comment         0 : { *(.comment) }
  .gnu.build.attributes : { *(.gnu.build.attributes .gnu.build.attributes.*) }
  /* DWARF 1 */
  .debug           0 : { *(.debug) }
  .line            0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo   0 : { *(.debug_srcinfo) }
  .debug_sfnames   0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges   0 : { *(.debug_aranges) }
  .debug_pubnames  0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info      0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev    0 : { *(.debug_abbrev) }
  .debug_line      0 : { *(.debug_line .debug_line.* .debug_line_end) }
  .debug_frame     0 : { *(.debug_frame) }
  .debug_str       0 : { *(.debug_str) }
  .debug_loc       0 : { *(.debug_loc) }
  .debug_macinfo   0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  /* DWARF 3 */
  .debug_pubtypes  0 : { *(.debug_pubtypes) }
  .debug_ranges    0 : { *(.debug_ranges) }
  /* DWARF Extension.  */
  .debug_macro     0 : { *(.debug_macro) }
  .debug_addr      0 : { *(.debug_addr) }
  .gnu.attributes  0 : { KEEP (*(.gnu.attributes)) }
  /DISCARD/          : { *(.note.GNU-stack) *(.gnu_debuglink) *(.gnu.lto_*) }

/* ************************************************************************************************* */
/* Export symbols for neorv32 crt0 start-up code                                                     */
/* ************************************************************************************************* */
  PROVIDE(__crt0_max_heap            = __neorv32_heap_size);
  PROVIDE(__crt0_ram_last            = (ORIGIN(ram) + LENGTH(ram)) - 1);
  PROVIDE(__crt0_bss_start           = ADDR(.bss));
  PROVIDE(__crt0_bss_end             = ADDR(.bss) + SIZEOF(.bss));
  PROVIDE(__crt0_copy_data_src_begin = LOADADDR(.data));
  PROVIDE(__crt0_copy_data_dst_begin = ADDR(.data));
  PROVIDE(__crt0_copy_data_dst_end   = ADDR(.data) + SIZEOF(.data));
  PROVIDE(__crt0_rom_base            = ORIGIN(rom));
  PROVIDE(__crt0_rom_size            = LENGTH(rom));
  PROVIDE(__crt0_ram_base            = ORIGIN(ram));
  PROVIDE(__crt0_ram_size            = LENGTH(ram));
}
