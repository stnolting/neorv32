/**
 * @file spinlock.h
 * @brief Single simple spin-lock based on atomic lr/sc operations.
 */

#ifndef spinlock_h
#define spinlock_h

void spin_lock(void);
void spin_unlock(void);

#endif // spinlock_h
