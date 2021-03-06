OUTPUT_ARCH( "riscv" )
ENTRY(_start)

SECTIONS
{

  . = 0x00000000;
  .text : {
    *(.text.trap)
    *(.text.init)

    . = ALIGN(0x1000);
    *(.tohost)

    . = ALIGN(0x1000);
    *(.text)
    _etext = .;

    *(.data)
    *(.data.string)
    _edata = .;

    *(.bss)
    _end = .;

  }

}
