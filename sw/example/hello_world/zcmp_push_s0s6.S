# RISC-V Assembly function using ZCMP extension: int zcmp_test_push(int a, int b)
# Uses cm.push to save s0-s6 registers and adjust stack pointer

# Function signature: int zcmp_test_push(int a, int b)
# Parameters: a0 = a, a1 = b
# Return: a0 = a + b

    .global zcmp_push_s0s6
    .type   zcmp_push_s0s6, @function

zcmp_push_s0s6:

    cm.push {ra, s0-s6}, -64  

    add     a0, a0, a1                      
    lw      ra, 32(sp)
    lw      s0, 36(sp)
    lw      s1, 40(sp)
    lw      s2, 44(sp)
    lw      s3, 48(sp)
    lw      s4, 52(sp)
    lw      s5, 56(sp)
    lw      s6, 60(sp)

# Free the 64-byte frame
    addi    sp, sp, 64

# Return (ra restored above)
    ret

    .size   zcmp_push_s0s6, .-zcmp_push_s0s6
