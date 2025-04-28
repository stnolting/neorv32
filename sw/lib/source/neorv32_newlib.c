// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_newlib.c
 * @brief NEORV32-specific Newlib system calls
 * @note Sources:
 * https://www.sourceware.org/newlib/libc.html#Syscalls
 * https://interrupt.memfault.com/blog/boostrapping-libc-with-newlib
 */

#include <neorv32.h>
#include <newlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/time.h>

// global error variable
#include <errno.h>
#undef errno
extern int errno;


 /**********************************************************************//**
 * Exit a program without cleaning up anything.
 **************************************************************************/
void _exit(int status) {

  (void)status;

  // jump to crt0's exit code
  asm volatile (
    ".extern __crt0_main_exit \n"
    "la ra,  __crt0_main_exit \n"
    "jr ra                    \n"
  );

  // will never be reached
  __builtin_unreachable();
  while(1);
}


 /**********************************************************************//**
 * Close file handle.
 **************************************************************************/
int _close(int file) {
  (void)file;
  return -1; // no files available
}


 /**********************************************************************//**
 * Status of an open file. All files are regarded as character special devices.
 **************************************************************************/
int _fstat(int file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}


 /**********************************************************************//**
 * Process-ID; this is sometimes used to generate strings unlikely to
 * conflict with other processes.
 **************************************************************************/
int _getpid() {
  return 1; // there is only one process by default
}


 /**********************************************************************//**
 * Query whether output stream is a terminal.
 * We only support terminal outputs here.
 **************************************************************************/
int _isatty(int file) {
  (void)file;
  return 1;
}


 /**********************************************************************//**
 * Send a signal.
 **************************************************************************/
int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}


 /**********************************************************************//**
 * Set position in a file.
 **************************************************************************/
int _lseek(int file, int ptr, int dir) {
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}


 /**********************************************************************//**
 * Read from a file. STDIN will read from UART0, all other input streams
 * will read from UART1.
 **************************************************************************/
int _read(int file, char *ptr, int len) {

  int read_cnt = 0;

  // read STDIN stream from NEORV32.UART0 (if available)
  if ((file == STDIN_FILENO) && (neorv32_uart_available(NEORV32_UART0))) {
    while (len--) {
      *ptr++ = (char)neorv32_uart_getc(NEORV32_UART0);
      read_cnt++;
    }
    return read_cnt;
  }
  // read all other input streams from NEORV32.UART1 (if available)
  else if (neorv32_uart_available(NEORV32_UART1)) {
    while (len--) {
      *ptr++ = (char)neorv32_uart_getc(NEORV32_UART1);
      read_cnt++;
    }
    return read_cnt;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
}


 /**********************************************************************//**
 * Write to a file. STDOUT and STDERR will write to UART0, all other
 * output streams will write to UART1.
 **************************************************************************/
int _write(int file, char *ptr, int len) {

  int write_cnt = 0;

  // write STDOUT and STDERR streams to NEORV32.UART0 (if available)
  if ((file == STDOUT_FILENO) || (file == STDERR_FILENO)) {
    if (neorv32_uart_available(NEORV32_UART0)) {
      while (len--) {
        neorv32_uart_putc(NEORV32_UART0, *ptr++);
        write_cnt++;
      }
      return write_cnt;
    }
    else {
      errno = ENOSYS;
      return -1;
    }
  }

  // write all other output streams to NEORV32.UART1 (if available)
  if (neorv32_uart_available(NEORV32_UART1)) {
    while (len--) {
      neorv32_uart_putc(NEORV32_UART1, *ptr++);
      write_cnt++;
    }
    return write_cnt;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
}


 /**********************************************************************//**
 * Dynamic memory management. Used by "malloc" and "free", among others.
 **************************************************************************/
void *_sbrk(int incr) {

  static unsigned char *curr_heap_ptr = NULL; // current heap pointer
  unsigned char *prev_heap_ptr; // previous heap pointer

  // initialize
  if (curr_heap_ptr == NULL) {
    curr_heap_ptr = (unsigned char *)NEORV32_HEAP_BEGIN;
  }

  // do we have a heap at all?
  if ((NEORV32_HEAP_BEGIN == NEORV32_HEAP_END) || (NEORV32_HEAP_SIZE == 0)) {
    write(STDERR_FILENO, "[neorv32-newlib] no heap available\r\n", 36);
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  // sufficient space left?
  if ((((uint32_t)curr_heap_ptr) + ((uint32_t)incr)) >= NEORV32_HEAP_END) {
    write(STDERR_FILENO, "[neorv32-newlib] heap exhausted\r\n", 33);
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  // runtime stack collision?
  register uint32_t stack_pntr asm("sp");
  asm volatile ("" : "=r" (stack_pntr));
  if ((((uint32_t)curr_heap_ptr) + ((uint32_t)incr)) >= stack_pntr) {
    write(STDERR_FILENO, "[neorv32-newlib] heap/stack collision\r\n", 39);
    errno = ENOMEM;
    exit(-911);
    return (void*)-1; // error - no more memory
  }

  prev_heap_ptr = curr_heap_ptr;
  curr_heap_ptr += incr;

  return (void*)prev_heap_ptr;
}


 /**********************************************************************//**
 * Get Unix time. Used by "time", among others.
 **************************************************************************/
int _gettimeofday(struct timeval *tv) {

  // use MTIME as system time (if available)
  if (neorv32_clint_available()) {
    tv->tv_sec  = (time_t)neorv32_clint_unixtime_get();
    tv->tv_usec = (suseconds_t)(tv->tv_sec * 1000000);
    return 0;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
}
