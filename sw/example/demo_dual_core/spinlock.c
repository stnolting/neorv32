/**
 * @file spinlock.c
 * @brief Single simple spinlock based on atomic memory operations.
 */
#include <neorv32.h>

/**********************************************************************//**
 * Private spinlock locked variable.
 **************************************************************************/
static volatile uint32_t __spin_locked = 0;


/**********************************************************************//**
 * Spinlock: set lock.
 *
 * @warning This function is blocking until the lock is acquired and set.
 **************************************************************************/
void spin_lock(void) {

  while(!__sync_bool_compare_and_swap(&__spin_locked, 0, -1)); // -> lr/sc
}


/**********************************************************************//**
 * Spinlock: remove lock.
 **************************************************************************/
void spin_unlock(void) {

  while(!__sync_bool_compare_and_swap(&__spin_locked, -1, 0)); // -> lr/sc
}
