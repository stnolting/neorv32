# RISC-V Assembly function: int zcmp_test(int a, int b)
# Saves s0-s5 registers, adds a and b, then restores registers
#
# Function signature: int zcmp_test(int a, int b)
# Parameters: a0 = a, a1 = b
# Return: a0 = a + b

.text
.global zcmp_test
.type zcmp_test, @function

zcmp_test:
    # Save registers s0-s5 to stack and adjust stack pointer
    # Need to save 6 registers (s0-s5) = 6 * 4 bytes = 24 bytes
    addi sp, sp, -24        # Adjust stack pointer (reserve 24 bytes)
    
    # Save registers to stack
    sw   s0, 0(sp)          # Save s0 at sp+0
    sw   s1, 4(sp)          # Save s1 at sp+4
    sw   s2, 8(sp)          # Save s2 at sp+8
    sw   s3, 12(sp)         # Save s3 at sp+12
    sw   s4, 16(sp)         # Save s4 at sp+16
    sw   s5, 20(sp)         # Save s5 at sp+20
    
    # Function body: add a and b
    # a0 contains parameter 'a', a1 contains parameter 'b'
    add  a0, a0, a1         # a0 = a + b (result in a0 for return)
    
    # Restore registers from stack
    lw   s0, 0(sp)          # Restore s0 from sp+0
    lw   s1, 4(sp)          # Restore s1 from sp+4
    lw   s2, 8(sp)          # Restore s2 from sp+8
    lw   s3, 12(sp)         # Restore s3 from sp+12
    lw   s4, 16(sp)         # Restore s4 from sp+16
    lw   s5, 20(sp)         # Restore s5 from sp+20
    
    # Restore stack pointer
    addi sp, sp, 24         # Restore stack pointer (free 24 bytes)
    
    # Return to caller
    ret                     # Return (a0 contains the result)

.size zcmp_test, .-zcmp_test

# RISC-V Assembly function using ZCMP extension: int zcmp_test_push(int a, int b)
# Uses cm.push to save s0-s5 registers and adjust stack pointer

# Function signature: int zcmp_test_push(int a, int b)
# Parameters: a0 = a, a1 = b
# Return: a0 = a + b

.global zcmp_test_push
.type zcmp_test_push, @function

zcmp_test_push:
    # Save registers s0-s5 to stack using ZCMP instruction
    # cm.push saves the specified registers and adjusts stack pointer
    cm.push {ra, s0-s5}, -64    # Push s0-s5 registers and adjust sp by -24
    
    # Function body: add a and b
    # a0 contains parameter 'a', a1 contains parameter 'b'
    add  a0, a0, a1         # a0 = a + b (result in a0 for return)
    
    # cm.push stores at oldsp-4..-28 and then sets sp = oldsp-64
    # so reload using offsets relative to the new sp:
    lw    ra, 36(sp)     # ra was at oldsp-28  => (sp+64)-28 = sp+36
    lw    s0, 40(sp)     # s0 was at oldsp-24  => sp+40
    lw    s1, 44(sp)     # s1 was at oldsp-20  => sp+44
    lw    s2, 48(sp)     # s2 was at oldsp-16  => sp+48
    lw    s3, 52(sp)     # s3 was at oldsp-12  => sp+52
    lw    s4, 56(sp)     # s4 was at oldsp-8   => sp+56
    lw    s5, 60(sp)     # s5 was at oldsp-4   => sp+60

    # Free the 64-byte frame
    addi  sp, sp, 64

    # Return (ra restored above)
    ret

.size zcmp_test_push, .-zcmp_test_push
