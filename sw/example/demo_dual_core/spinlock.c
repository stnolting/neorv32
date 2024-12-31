/**
 * @file spinlock.c
 * @brief Single simple spin-lock based on atomic lr/sc operations.
 */
#include <neorv32.h>

/**********************************************************************//**
 * Private spinlock locked variable. We can only use a single spinlock
 * as the processor only features a single reservation set.
 **************************************************************************/
static volatile uint32_t __spin_locked = 0;


/**********************************************************************//**
 * Spinlock: set lock.
 *
 * @warning This function is blocking until the lock is acquired.
 **************************************************************************/
void spin_lock(void) {

  while (neorv32_cpu_amoswapw((uint32_t)&__spin_locked, 1) != 0);
}


/**********************************************************************//**
 * Spinlock: remove lock.
 **************************************************************************/
void spin_unlock(void) {

  neorv32_cpu_amoswapw((uint32_t)&__spin_locked, 0);
}
