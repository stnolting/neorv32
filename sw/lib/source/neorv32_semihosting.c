// ================================================================================ //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

/**
 * @file neorv32_semihosting.c
 * @brief RISC-V semihosting source file.
 */

#include <neorv32.h>
#include <string.h>

/**********************************************************************//**
 * Print single character to host's STDOUT.
 *
 * @param[in] c Character to print.
 **************************************************************************/
void neorv32_semihosting_putc(char c) {
  neorv32_semihosting_req(SEMIHOSTING_SYS_WRITEC, (void*)&c);
}


/**********************************************************************//**
 * Print zero-terminated string to host's STDOUT.
 *
 * @param[in] pnt Pointer to zero-terminated string.
 **************************************************************************/
void neorv32_semihosting_puts(const char* pnt) {
  neorv32_semihosting_req(SEMIHOSTING_SYS_WRITE0, (void*)pnt);
}


/**********************************************************************//**
 * Read single character from host's STDIN.
 *
 * @warning This function is blocking.
 *
 * @return Received character.
 **************************************************************************/
char neorv32_semihosting_getc(void) {
  return (char)neorv32_semihosting_req(SEMIHOSTING_SYS_READC, NULL);
}


/**********************************************************************//**
 * Write buffer to host's file ID.
 *
 * @param[in] file File descriptor (stream; should be 1 = STDOUT).
 * @param[in] buffer Pointer to data buffer.
 * @param[in] len Length of data to write.
 * @return Number of characters NOT send.
 **************************************************************************/
int neorv32_semihosting_write(int file, char *buffer, int len) {
  uint32_t args[3];
  args[0] = (uint32_t)file;
  args[1] = (uint32_t)buffer;
  args[2] = (uint32_t)len;
  return len - (int)neorv32_semihosting_req(SEMIHOSTING_SYS_WRITE, (void*)args);
}


/**********************************************************************//**
 * Read buffer from host's file ID.
 *
 * @param[in] file File descriptor (stream; should be 0 = STDIN).
 * @param[in] buffer Pointer to data buffer.
 * @param[in] len Length of data to read.
 * @return Number of characters NOT read.
 **************************************************************************/
int neorv32_semihosting_read(int file, char *buffer, int len) {
  uint32_t args[3];
  args[0] = (uint32_t)file;
  args[1] = (uint32_t)buffer;
  args[2] = (uint32_t)len;
  return len - (int)neorv32_semihosting_req(SEMIHOSTING_SYS_READ, (void*)args);
}


/**********************************************************************//**
 * Get host's current system time.
 *
 * @return Unix timestamp (time in seconds since Jan 1st 1970)
 **************************************************************************/
uint32_t neorv32_semihosting_time(void) {
  return (uint32_t)neorv32_semihosting_req(SEMIHOSTING_SYS_TIME, NULL);
}


/**********************************************************************//**
 * Get host's current system time.
 *
 * @warning Be careful!
 *
 * @param[in] buffer Pointer to data buffer.
 * @return Host command's exit status.
 **************************************************************************/
int neorv32_semihosting_cmd(char *buffer) {
  uint32_t args[2];
  args[0] = (uint32_t)buffer;
  args[1] = (uint32_t)strlen(buffer);
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_SYSTEM, (void*)args);
}
