    .global zcmp_push_pop
    .type   zcmp_push_pop, @function

zcmp_push_pop:

    cm.push {ra,s0-s6}, -64

    add     a0, a0, a1

    cm.popretz {ra, s0-s6}, 64