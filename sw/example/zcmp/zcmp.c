#include "zcmp.h"
volatile uint32_t sp_save __attribute__((aligned(16)));
volatile uint32_t test_frame[FRAME_SIZE_WORDS] __attribute__((aligned(16)));
volatile uint32_t test_results[N_RESULTS] __attribute__((aligned(16)));