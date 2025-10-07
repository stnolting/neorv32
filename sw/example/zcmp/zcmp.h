#pragma once

#include <neorv32.h>

#define FRAME_SIZE_WORDS 64

extern volatile uint32_t sp_save;
extern volatile uint32_t test_frame[FRAME_SIZE_WORDS];