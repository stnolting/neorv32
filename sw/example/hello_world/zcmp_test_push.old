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
    cm.push {ra, s0-s5}, -64    # Push s0-s5 registers and adjust sp by -64
    
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
