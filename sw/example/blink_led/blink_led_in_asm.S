.file	"blink_led_in_asm.S"
.section .text
.balign 4
.global blink_led_asm

blink_led_asm:

  /* base address of GPIO controller's output port is passed as argument (in a0)*/
  sw   zero, 0(a0) /* clear output port */

  li   t1, 0 /* initialize counter */


blink_loop:
  andi t1, t1, 255 /* apply 8-bit mask */
  sw   t1, 0(a0)   /* output current counter value */
  addi t1, t1, 1   /* increment counter */

  jal  ra, blink_delay /* call delay function */

  j    blink_loop


blink_delay:
  li   t2, 0xfffff /* delay time */

blink_delay_loop:
  beq  t2, zero, blink_delay_end
  addi t2, t2, -1
  nop
  nop
  j    blink_delay_loop

blink_delay_end:
  ret

.end
