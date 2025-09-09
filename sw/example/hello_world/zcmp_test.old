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