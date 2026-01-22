OUTPUT_ARCH( "riscv" )
ENTRY(rvtest_entry_point)

SECTIONS
{
  . = 0x80000000;
  .text :
  {
    *(.text.init)
    *(.text)
    *(.data)
    *(.data.string)
    *(.bss)
    _end = .;
  }
}
