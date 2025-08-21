// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_semihosting.h
 * @brief RISC-V semihosting header file.
 */

#ifndef NEORV32_SEMIHOSTING_H
#define NEORV32_SEMIHOSTING_H

#include <stdint.h>


/**********************************************************************//**
 * @name Host service request IDs
 * See https://developer.arm.com/documentation/dui0203/j/semihosting/semihosting-operations
 * for the description.
 **************************************************************************/
/**@{*/
enum SEMIHOSTING_SYS_enum {
  SEMIHOSTING_SYS_OPEN        = 0x01, /**< open a file */
  SEMIHOSTING_SYS_CLOSE       = 0x02, /**< close a file */
  SEMIHOSTING_SYS_WRITEC      = 0x03, /**< write a character byte, pointed */
  SEMIHOSTING_SYS_WRITE0      = 0x04, /**< writes a null terminated string */
  SEMIHOSTING_SYS_WRITE       = 0x05, /**< write data to a file */
  SEMIHOSTING_SYS_READ        = 0x06, /**< read data from a file */
  SEMIHOSTING_SYS_READC       = 0x07, /**< read a character from stdin */
  SEMIHOSTING_SYS_ISTTY       = 0x09, /**< check if it is a TTY */
  SEMIHOSTING_SYS_SEEK        = 0x0A, /**< move current file position */
  SEMIHOSTING_SYS_FLEN        = 0x0C, /**< tell the file length */
  SEMIHOSTING_SYS_TMPNAME     = 0x0D, /**< get a temporary file handle */
  SEMIHOSTING_SYS_REMOVE      = 0x0E, /**< remove a file */
  SEMIHOSTING_SYS_RENAME      = 0x0F, /**< rename a file */
  SEMIHOSTING_SYS_CLOCK       = 0x10, /**< returns the number of centi-seconds since execution started */
  SEMIHOSTING_SYS_TIME        = 0x11, /**< time in seconds since Jan 1st 1970 */
  SEMIHOSTING_SYS_SYSTEM      = 0x12, /**< passes a command to the host command-line interpreter */
  SEMIHOSTING_SYS_ERRNO       = 0x13, /**< get the current error number */
  SEMIHOSTING_SYS_GET_CMDLINE = 0x15, /**< returns the command line used for the call to the executable */
  SEMIHOSTING_SYS_HEAPINFO    = 0x16, /**< returns the system stack and heap parameters */
  SEMIHOSTING_SYS_EXCEPTION   = 0x18, /**< exit application */
  SEMIHOSTING_SYS_ELLAPSED    = 0x30, /**< returns the number of elapsed target ticks since execution started */
  SEMIHOSTING_SYS_TICKFREQ    = 0x31  /**< returns the tick frequency */
};
/**@}*/


/**********************************************************************//**
 * @name Semihosting "open" modes
 * Source: https://docs.zephyrproject.org/apidoc/latest/group__semihost.html
 **************************************************************************/
/**@{*/
enum SEMIHOST_OPEN_MODE_enum {
  SEMIHOSTING_OPEN_R       = 0,
  SEMIHOSTING_OPEN_RB      = 1,
  SEMIHOSTING_OPEN_R_PLUS  = 2,
  SEMIHOSTING_OPEN_RB_PLUS = 3,
  SEMIHOSTING_OPEN_W       = 4,
  SEMIHOSTING_OPEN_WB      = 5,
  SEMIHOSTING_OPEN_W_PLUS  = 6,
  SEMIHOSTING_OPEN_WB_PLUS = 7,
  SEMIHOSTING_OPEN_A       = 8,
  SEMIHOSTING_OPEN_AB      = 9,
  SEMIHOSTING_OPEN_A_PLUS  = 10,
  SEMIHOSTING_OPEN_AB_PLUS = 11
};
/**@}*/


/**********************************************************************//**
 * @name Prototypes
 **************************************************************************/
/**@{*/
void neorv32_semihosting_putc(char c);
void neorv32_semihosting_puts(const char* pnt);
char neorv32_semihosting_getc(void);
//
int neorv32_semihosting_open(char *path, int mode);
int neorv32_semihosting_close(int file);
int neorv32_semihosting_write(int file, char *buffer, int len);
int neorv32_semihosting_read(int file, char *buffer, int len);
int neorv32_semihosting_istty(int file);
int neorv32_semihosting_seek(int file, int pos);
int neorv32_semihosting_flen(int file);
int neorv32_semihosting_time(void);
int neorv32_semihosting_system(char *cmd);
/**@}*/


/**********************************************************************//**
 * Send a semihosting request to the host.
 *
 * @param[in] id Service request ID (#SEMIHOSTING_SYS_enum).
 * @param[in] arg Argument / pointer to data array.
 * @return Host return value.
 **************************************************************************/
inline int __attribute__ ((always_inline)) neorv32_semihosting_req(int id, void* arg) {

  register int value asm ("a0") = id;
  register void* data asm ("a1") = arg;
  asm volatile (
    " .option push      \n"
    " .option norvc     \n" // we need 32-bit instruction words here
    " .balign 4         \n" // this has to be aligned
    " slli x0, x0, 0x1f \n" // magic triplet: entry NOP
    " ebreak            \n" // magic triplet: break to debugger
    " srai x0, x0, 0x07 \n" // magic triplet: exit NOP
    " .option pop       \n"
    : "=r" (value) : "0" (value), "r" (data) : "memory"
  );
  return value;
}


#endif // NEORV32_SEMIHOSTING_H
