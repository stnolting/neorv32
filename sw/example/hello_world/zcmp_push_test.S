# RISC-V Assembly function using ZCMP extension: int zcmp_test_push(int a, int b)
# Uses cm.push to save s0-s6 registers and adjust stack pointer

# Function signature: int zcmp_test_push(int a, int b)
# Parameters: a0 = a, a1 = b
# Return: a0 = a + b

# .global zcmp_push_s0s6
# .type zcmp_push_s0s6, @function

# zcmp_push_s0s6:

# cm.push {ra, s0-s6}, -64

# add a0, a0, a1
# lw ra, 32(sp)
# lw s0, 36(sp)
# lw s1, 40(sp)
# lw s2, 44(sp)
# lw s3, 48(sp)
# lw s4, 52(sp)
# lw s5, 56(sp)
# lw s6, 60(sp)

# # Free the 64-byte frame
# addi sp, sp, 64

# # Return (ra restored above)
# ret

# .global zcmp_push_s0s5
# .type zcmp_push_s0s5, @function

#---------------------------------------------------------------------------#

    .global zcmp_push
    .type   zcmp_push, @function

zcmp_push:

    cm.push {ra}, -16

    add     a0, a0, a1
    lw      ra, 12(sp)
    addi    sp, sp, 16

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0
    .type   zcmp_push_s0, @function

zcmp_push_s0:

    cm.push {ra,s0}, -16

    add     a0, a0, a1
    lw      ra, 8(sp)
    lw      s0, 12(sp)
    addi    sp, sp, 16

    ret


#---------------------------------------------------------------------------#

    .global zcmp_push_s0s1
    .type   zcmp_push_s0s1, @function

zcmp_push_s0s1:

    cm.push {ra,s0-s1}, -32

    add     a0, a0, a1
    lw      ra, 20(sp)
    lw      s0, 24(sp)
    lw      s1, 28(sp)
    addi    sp, sp, 32

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s2
    .type   zcmp_push_s0s2, @function

zcmp_push_s0s2:

    cm.push {ra,s0-s2}, -32

    add     a0, a0, a1
    lw      ra, 16(sp)
    lw      s0, 20(sp)
    lw      s1, 24(sp)
    lw      s2, 28(sp)
    addi    sp, sp, 32

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s3
    .type   zcmp_push_s0s3, @function

zcmp_push_s0s3:

    cm.push {ra,s0-s3}, -32

    add     a0, a0, a1
    lw      ra, 12(sp)
    lw      s0, 16(sp)
    lw      s1, 20(sp)
    lw      s2, 24(sp)
    lw      s3, 28(sp)
    addi    sp, sp, 32

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s4
    .type   zcmp_push_s0s4, @function

zcmp_push_s0s4:

    cm.push {ra,s0-s4}, -48

    add     a0, a0, a1
    lw      ra, 24(sp)
    lw      s0, 28(sp)
    lw      s1, 32(sp)
    lw      s2, 36(sp)
    lw      s3, 40(sp)
    lw      s4, 44(sp)
    addi    sp, sp, 48

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s5
    .type   zcmp_push_s0s5, @function

zcmp_push_s0s5:

    cm.push {ra,s0-s5}, -48

    add     a0, a0, a1
    lw      ra, 20(sp)
    lw      s0, 24(sp)
    lw      s1, 28(sp)
    lw      s2, 32(sp)
    lw      s3, 36(sp)
    lw      s4, 40(sp)
    lw      s5, 44(sp)
    addi    sp, sp, 48

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s6
    .type   zcmp_push_s0s6, @function

zcmp_push_s0s6:

    cm.push {ra,s0-s6}, -64

    add     a0, a0, a1
    lw      ra, 32(sp)
    lw      s0, 36(sp)
    lw      s1, 40(sp)
    lw      s2, 44(sp)
    lw      s3, 48(sp)
    lw      s4, 52(sp)
    lw      s5, 56(sp)
    lw      s6, 60(sp)
    addi    sp, sp, 64

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s7
    .type   zcmp_push_s0s7, @function

zcmp_push_s0s7:

    cm.push {ra,s0-s7}, -64

    add     a0, a0, a1
    lw      ra, 28(sp)
    lw      s0, 32(sp)
    lw      s1, 36(sp)
    lw      s2, 40(sp)
    lw      s3, 44(sp)
    lw      s4, 48(sp)
    lw      s5, 52(sp)
    lw      s6, 56(sp)
    lw      s7, 60(sp)
    addi    sp, sp, 64

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s8
    .type   zcmp_push_s0s8, @function

zcmp_push_s0s8:

    cm.push {ra,s0-s8}, -64

    add     a0, a0, a1
    lw      ra, 24(sp)
    lw      s0, 28(sp)
    lw      s1, 32(sp)
    lw      s2, 36(sp)
    lw      s3, 40(sp)
    lw      s4, 44(sp)
    lw      s5, 48(sp)
    lw      s6, 52(sp)
    lw      s7, 56(sp)
    lw      s8, 60(sp)
    addi    sp, sp, 64

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s9
    .type   zcmp_push_s0s9, @function

zcmp_push_s0s9:

    cm.push {ra,s0-s9}, -64

    add     a0, a0, a1
    lw      ra, 20(sp)
    lw      s0, 24(sp)
    lw      s1, 28(sp)
    lw      s2, 32(sp)
    lw      s3, 36(sp)
    lw      s4, 40(sp)
    lw      s5, 44(sp)
    lw      s6, 48(sp)
    lw      s7, 52(sp)
    lw      s8, 56(sp)
    lw      s9, 60(sp)
    addi    sp, sp, 64

    ret

#---------------------------------------------------------------------------#

    .global zcmp_push_s0s11
    .type   zcmp_push_s0s11, @function

zcmp_push_s0s11:

    cm.push {ra,s0-s11}, -64

    add     a0, a0, a1
    lw      ra, 12(sp)
    lw      s0, 16(sp)
    lw      s1, 20(sp)
    lw      s2, 24(sp)
    lw      s3, 28(sp)
    lw      s4, 32(sp)
    lw      s5, 36(sp)
    lw      s6, 40(sp)
    lw      s7, 44(sp)
    lw      s8, 48(sp)
    lw      s9, 52(sp)
    lw      s10, 56(sp)
    lw      s11, 60(sp)
    addi    sp, sp, 64

    ret
#---------------------------------------------------------------------------#

# .global zcmp_push_s0s5
# .type zcmp_push_s0s5, @function

# zcmp_push_s0s5:

# cm.push {ra, s0-s5}, -64

# add a0, a0, a1
# lw ra, 36(sp)
# lw s0, 40(sp)
# lw s1, 44(sp)
# lw s2, 48(sp)
# lw s3, 52(sp)
# lw s4, 56(sp)
# lw s5, 60(sp)

# # Free the 64-byte frame
# addi sp, sp, 64

# # Return (ra restored above)
# ret

    .global zcmp_test_push
    .type   zcmp_test_push, @function

zcmp_test_push:
# Save registers s0-s5 to stack using ZCMP instruction
# cm.push saves the specified registers and adjusts stack pointer
    cm.push {ra, s0-s5}, -64          # Push s0-s5 registers and adjust sp by -64

# Function body: add a and b
# a0 contains parameter 'a', a1 contains parameter 'b'
    add     a0, a0, a1                # a0 = a + b (result in a0 for return)

# cm.push stores at oldsp-4..-28 and then sets sp = oldsp-64
# so reload using offsets relative to the new sp:
    lw      ra, 36(sp)                # ra was at oldsp-28 => (sp+64)-28 = sp+36
    lw      s0, 40(sp)                # s0 was at oldsp-24 => sp+40
    lw      s1, 44(sp)                # s1 was at oldsp-20 => sp+44
    lw      s2, 48(sp)                # s2 was at oldsp-16 => sp+48
    lw      s3, 52(sp)                # s3 was at oldsp-12 => sp+52
    lw      s4, 56(sp)                # s4 was at oldsp-8 => sp+56
    lw      s5, 60(sp)                # s5 was at oldsp-4 => sp+60

# Free the 64-byte frame
    addi    sp, sp, 64

# Return (ra restored above)
    ret


