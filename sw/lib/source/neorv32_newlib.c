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
 * https://www.embecosm.com/appnotes/ean9/ean9-howto-newlib-1.0.pdf
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

// global environment
char *__env[1] = { 0 };
char **environ = __env;


/**********************************************************************//**
 * Issue a warning when semihosting is enabled.
 **************************************************************************/
/**@{*/
#ifdef STDIO_SEMIHOSTING
  #warning Newlib/stdio.h semihosting enabled.
#endif
#ifdef TIME_SEMIHOSTING
  #warning Newlib/time.h semihosting enabled.
#endif
/**@}*/


 /**********************************************************************//**
 * Exit a program without cleaning up anything.
 **************************************************************************/
void _exit(int status) {

  // put status into register 'a0' and jump to crt0's exit code
  asm volatile (
    ".extern __crt0_main_exit \n"
    "la ra,  __crt0_main_exit \n"
    "mv a0,  %[exit_code]     \n"
    "jr ra                    \n"
    : : [exit_code] "r" (status)
  );

  // will never be reached
  __builtin_unreachable();
  while(1);
}


 /**********************************************************************//**
 * Open file handle.
 **************************************************************************/
int _open(char *pathname, int flags) {
#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_open(pathname, flags);
#else
  (void)pathname;
  (void)flags;
  return -1; // no files available
#endif
}


 /**********************************************************************//**
 * Close file handle.
 **************************************************************************/
int _close(int file) {
#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_close(file);
#else
  (void)file;
  return -1; // no files available
#endif
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
 * Create new process.
 **************************************************************************/
int _fork(void) {
  errno = EAGAIN;
  return -1;
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
 **************************************************************************/
int _isatty(int file) {
#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_istty(file);
#else
  (void)file;
  return 1; // all streams are terminals
#endif
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
 * Rename existing file.
 **************************************************************************/
int _link(char *old_name, char *new_name) {
  (void)old_name;
  (void)new_name;
  errno = EMLINK;
  return -1;
}


 /**********************************************************************//**
 * Set position in a file.
 **************************************************************************/
int _lseek(int file, int ptr, int dir) {
#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_seek(file, ptr);
#else
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
#endif
}


 /**********************************************************************//**
 * Status of a file.
 **************************************************************************/
int _stat(char *file, struct stat *st) {
  (void)file;
  st->st_mode = S_IFCHR; // all files are character special devices
  return 0;
}


 /**********************************************************************//**
 * Wait for child process.
 **************************************************************************/
int _wait(int status) {
  (void)status;
  errno = ECHILD;
  return -1;
}


 /**********************************************************************//**
 * Remove a file's directory entry.
 **************************************************************************/
int _unlink(char *name) {
  (void)name;
  errno = ENOENT;
  return -1;
}


 /**********************************************************************//**
 * Read from a file. STDIN will read from UART0, all other input streams
 * will read from UART1.
 **************************************************************************/
int _read(int file, char *ptr, int len) {

#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_read(file, ptr, len);
#else
  char c = 0;
  int read_cnt = 0;

  // read STDIN stream from NEORV32.UART0 (if available)
  if ((file == STDIN_FILENO) && (neorv32_uart_available(NEORV32_UART0))) {
    while (len--) {
      c = (char)neorv32_uart_getc(NEORV32_UART0);
      *ptr++ = c;
      read_cnt++;
      if ((c == '\n') || (c == '\r')) { // also terminate on [press enter]
        return read_cnt;
      }
    }
    return read_cnt;
  }
  // read all other input streams from NEORV32.UART1 (if available)
  else if (neorv32_uart_available(NEORV32_UART1)) {
    while (len--) {
      c = (char)neorv32_uart_getc(NEORV32_UART1);
      *ptr++ = c;
      read_cnt++;
      if ((c == '\n') || (c == '\r')) { // also terminate on [press enter]
        return read_cnt;
      }
    }
    return read_cnt;
  }
  else {
    errno = ENOSYS;
    return -1;
  }
#endif
}


 /**********************************************************************//**
 * Write to a file. STDOUT and STDERR will write to UART0, all other
 * output streams will write to UART1.
 **************************************************************************/
int _write(int file, char *ptr, int len) {

#ifdef STDIO_SEMIHOSTING
  return neorv32_semihosting_write(file, ptr, len);
#else
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
#endif
}


 /**********************************************************************//**
 * Dynamic memory management. Used by "malloc" and "free", among others.
 **************************************************************************/
void *_sbrk(int incr) {

  static unsigned char *curr_heap = NULL; // current heap pointer
  unsigned char *prev_heap; // previous heap pointer

  // initialize
  if (curr_heap == NULL) {
    curr_heap = (unsigned char *)NEORV32_HEAP_BEGIN;
  }

  // do we have a heap at all?
  if ((NEORV32_HEAP_BEGIN == NEORV32_HEAP_END) || (NEORV32_HEAP_SIZE == 0)) {
#ifdef NEWLIB_DEBUG
    write(STDERR_FILENO, "[neorv32-newlib] no heap available\r\n", 36);
#endif
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  // sufficient space left?
  if ((uint32_t)(curr_heap + incr) >= NEORV32_HEAP_END) {
#ifdef NEWLIB_DEBUG
    write(STDERR_FILENO, "[neorv32-newlib] heap exhausted\r\n", 33);
#endif
    errno = ENOMEM;
    return (void*)-1; // error - no more memory
  }

  prev_heap = curr_heap;
  curr_heap += incr;

  return (void*)prev_heap;
}


 /**********************************************************************//**
 * Get Unix time. Used by "time", among others.
 **************************************************************************/
int _gettimeofday(struct timeval *tv) {
#ifdef TIME_SEMIHOSTING
  tv->tv_sec  = (time_t)neorv32_semihosting_time();
  tv->tv_usec = (suseconds_t)(tv->tv_sec * 1000000);
  return 0;
#else
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
#endif
}
