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
 * Open file on host system.
 *
 * @param[in] path Path/file name (zero-terminated string).
 * @param[in] mode File access mode (ISO C).
 * @return File handle.
 **************************************************************************/
int neorv32_semihosting_open(char *path, int mode) {
  uint32_t args[3];
  args[0] = (uint32_t)path;
  args[1] = (uint32_t)mode;
  args[2] = (uint32_t)strlen(path);
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_OPEN, (void*)args);
}


/**********************************************************************//**
 * Close file on host system.
 *
 * @param[in] file File handle.
 * @return  0 if the call is successful, -1 if the call is not successful.
 **************************************************************************/
int neorv32_semihosting_close(int file) {
  uint32_t args[1];
  args[0] = (uint32_t)file;
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_CLOSE, (void*)args);
}


/**********************************************************************//**
 * Write data buffer to host's file handle.
 *
 * @param[in] file File handle (stream; should be 1 = STDOUT).
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
 * Read data buffer from host's file handle.
 *
 * @param[in] file File handle (stream; should be 0 = STDIN).
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
 * Checks if a host file is connected to an interactive device.
 *
 * @param[in] file File handle.
 * @return 1 if the handle identifies an interactive device, 0 if the handle
 * identifies a file, a value other than 1 or 0 if an error occurs.
 **************************************************************************/
int neorv32_semihosting_istty(int file) {
  uint32_t args[1];
  args[0] = (uint32_t)file;
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_ISTTY, (void*)args);
}


/**********************************************************************//**
 * Seeks to a specified position in a file.
 *
 * @param[in] file File handle.
 * @param[in] pos Offset specified from the start of the file
 * @return 0 if the request is successful.
 **************************************************************************/
int neorv32_semihosting_seek(int file, int pos) {
  uint32_t args[2];
  args[0] = (uint32_t)file;
  args[1] = (uint32_t)pos;
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_SEEK, (void*)args);
}


/**********************************************************************//**
 * Returns the length of a specified file.
 *
 * @param[in] file File handle.
 * @return The current length of the file object, -1 if call fails.
 **************************************************************************/
int neorv32_semihosting_flen(int file) {
  uint32_t args[1];
  args[0] = (uint32_t)file;
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_FLEN, (void*)args);
}


/**********************************************************************//**
 * Get host's current system time.
 *
 * @return Unix timestamp (time in seconds since Jan 1st 1970)
 **************************************************************************/
int neorv32_semihosting_time(void) {
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_TIME, NULL);
}


/**********************************************************************//**
 * Execute command in host's console.
 *
 * @warning Be careful!
 *
 * @param[in] cmd Pointer to zero-terminated command string.
 * @return Host command's exit status.
 **************************************************************************/
int neorv32_semihosting_system(char *cmd) {
  uint32_t args[2];
  args[0] = (uint32_t)cmd;
  args[1] = (uint32_t)strlen(cmd);
  return (int)neorv32_semihosting_req(SEMIHOSTING_SYS_SYSTEM, (void*)args);
}
