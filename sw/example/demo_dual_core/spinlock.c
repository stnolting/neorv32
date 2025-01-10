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

  while(__sync_lock_test_and_set(&__spin_locked, -1)); // -> amoswap.w
}


/**********************************************************************//**
 * Spinlock: remove lock.
 **************************************************************************/
void spin_unlock(void) {

  //__sync_lock_release(&__spin_locked); // uses fence that is not required here
  __sync_lock_test_and_set(&__spin_locked, 0); // -> amoswap.w
}
