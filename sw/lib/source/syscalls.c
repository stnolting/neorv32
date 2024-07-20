// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file syscalls.c
 * @brief Newlib system calls
 *
 * @warning UART0 (if available) is used to read/write console data (STDIN, STDOUT, STDERR, ...).
 *
 * @note Original source file: https://github.com/openhwgroup/cv32e40p/blob/master/example_tb/core/custom/syscalls.c
 * @note More information was derived from: https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib#implementing-newlib
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <newlib.h>
#include <sys/stat.h>
#include <sys/timeb.h>
#include <sys/times.h>
#include <utime.h>
#include <unistd.h>
#include <errno.h>
#include <neorv32.h>

#undef errno
extern int errno;

// defined in sw/common/crt0.S
extern const volatile unsigned int __crt0_main_exit;

// linker script symbols
extern char __heap_start[];
extern char __heap_end[];
extern char __crt0_max_heap[];


void *_sbrk(int incr) {

  static unsigned char *curr_heap_ptr = NULL; // current heap pointer
  unsigned char *prev_heap_ptr; // previous heap pointer

  const uint32_t heap_begin = (uint32_t)&__heap_start[0];
  const uint32_t heap_end   = (uint32_t)&__heap_end[0];
  const uint32_t heap_size  = (uint32_t)&__crt0_max_heap[0];

  // initialize
  if (curr_heap_ptr == NULL) {
    curr_heap_ptr = (unsigned char *)heap_begin;
  }

  // do we have a heap at all?
  if ((heap_begin == heap_end) || (heap_size == 0)) {
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  // sufficient space left?
  if ((((uint32_t)curr_heap_ptr) + ((uint32_t)incr)) >= heap_end) {
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  prev_heap_ptr = curr_heap_ptr;
  curr_heap_ptr += incr;

  return (void*)prev_heap_ptr;
}


int _close(int file) {

  return -1;
}


int _fstat(int file, struct stat *st) {

  st->st_mode = S_IFCHR; // all files are "character special files"
  return 0;
}


int _isatty(int file) {

  return 1;
}


int _lseek(int file, int ptr, int dir) {

  return 0;
}


void _exit(int status) {

  // jump to crt0's shutdown code
  asm volatile ("la t0, __crt0_main_exit \n"
                "jr t0                   \n");
  while(1); // will never be reached
}


void _kill(int pid, int sig) {

  errno = EINVAL;
  return;
}


int _getpid() {

  return 1;
}


int _write(int file, char *ptr, int len) {

  // write everything (STDOUT, STDERR, ...) to NEORV32.UART0 (if available)
  const void *eptr = ptr + len;
  if (neorv32_uart0_available()) {
    while (ptr != eptr) {
      neorv32_uart0_putc(*(char *)(ptr++));
    }
    return len;
  }
  else {
    return 0; // nothing sent
  }
}


int _read(int file, char *ptr, int len) {
  int read_cnt = 0;

  // read everything (STDIN, ...) from NEORV32.UART0 (if available)
  if (neorv32_uart0_available()) {
    char *char_ptr;
    char_ptr = (char *)ptr;
    while (len > 0) {
      *char_ptr++ = (char)neorv32_uart0_getc();
      read_cnt++;
      len--;
    }
  }

  return read_cnt;
}


int _gettimeofday(struct timeval *tp, void *tzp) {

  // use MTIME as system time (if available)
  if (neorv32_mtime_available()) {
    tp->tv_sec = (long int)neorv32_mtime_get_unixtime();
    tp->tv_usec = 0;
    return 0;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
}
