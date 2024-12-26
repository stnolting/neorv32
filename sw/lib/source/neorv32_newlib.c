// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_newlib.c
 * @brief NEORV32-specific Newlib system calls
 *
 * @note Original source file: https://github.com/openhwgroup/cv32e40p/blob/master/example_tb/core/custom/syscalls.c
 * @note More information was derived from: https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib#implementing-newlib
 *
 * @see https://stnolting.github.io/neorv32/sw/files.html
 */

#include <neorv32.h>
#include <newlib.h>
#include <sys/stat.h>
#include <sys/timeb.h>
#include <sys/times.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>
#include <errno.h>

#undef errno
extern int errno;


void *_sbrk(int incr) {

  static unsigned char *curr_heap_ptr = NULL; // current heap pointer
  unsigned char *prev_heap_ptr; // previous heap pointer

  // initialize
  if (curr_heap_ptr == NULL) {
    curr_heap_ptr = (unsigned char *)neorv32_heap_begin_c;
  }

  // do we have a heap at all?
  if ((neorv32_heap_begin_c == neorv32_heap_end_c) || (neorv32_heap_size_c == 0)) {
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  // sufficient space left?
  if ((((uint32_t)curr_heap_ptr) + ((uint32_t)incr)) >= neorv32_heap_end_c) {
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

  return -1;
}


int _lseek(int file, int ptr, int dir) {

  return 0;
}


void _exit(int status) {

  // jump to crt0's shutdown code
  asm volatile (".extern __crt0_main_exit \n"
                "la t0, __crt0_main_exit  \n"
                "jr t0                    \n");

  // will never be reached
  __builtin_unreachable();
  while(1);
}


void _kill(int pid, int sig) {

  errno = EINVAL;
  return;
}


int _getpid() {

  return 1;
}


int _write(int file, char *ptr, int len) {

  int write_cnt = 0;
  char c = 0;

  // write everything (STDOUT, STDERR, ...) to NEORV32.UART0 (if available)
  if (neorv32_uart0_available()) {
    while (len > 0) {
      c = (char)*ptr++;
      if (c == '\n') { // convert \n to \r\n
        neorv32_uart0_putc('\r');
      }
      neorv32_uart0_putc(c);
      len--;
      write_cnt++;
    }
  }
  else {
    errno = ENOSYS;
  }

  return write_cnt;
}


int _read(int file, char *ptr, int len) {

  int read_cnt = 0;

  // read everything (STDIN, ...) from NEORV32.UART0 (if available)
  if (neorv32_uart0_available()) {
    while (len > 0) {
      *ptr++ = (char)neorv32_uart0_getc();
      len--;
      read_cnt++;
    }
  }
  else {
    errno = ENOSYS;
  }

  return read_cnt;
}


int _gettimeofday(struct timeval *tp, void *tzp) {

  // use MTIME as system time (if available)
  if (neorv32_clint_available()) {
    tp->tv_sec = (long int)neorv32_clint_unixtime_get();
    tp->tv_usec = 0;
    return 0;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
}
