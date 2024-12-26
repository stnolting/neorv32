// ================================================================================ //
// Executable memory image generator                                                //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2024 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// executable signature ("magic word")
const uint32_t signature = 0x4788CAFE;

// output file types (operation select)
enum operation_enum {
  OP_APP_BIN,
  OP_APP_VHD,
  OP_BLD_VHD,
  OP_RAW_HEX,
  OP_RAW_BIN,
  OP_RAW_COE,
  OP_RAW_MEM,
  OP_RAW_MIF
};

int main(int argc, char *argv[]) {

  if ((argc != 4) && (argc != 5)) {
    printf("NEORV32 executable image generator\n"
           "Three arguments are required.\n"
           "1st: Operation\n"
           " -app_bin : Generate application executable binary (binary file, little-endian, with header) \n"
           " -app_vhd : Generate application raw executable memory image (vhdl package body file, no header)\n"
           " -bld_vhd : Generate bootloader raw executable memory image (vhdl package body file, no header)\n"
           " -raw_hex : Generate application raw executable (ASCII hex file, no header)\n"
           " -raw_bin : Generate application raw executable (binary file, no header)\n"
           " -raw_coe : Generate application raw executable (COE file, no header)\n"
           " -raw_mem : Generate application raw executable (MEM file, no header)\n"
           " -raw_mif : Generate application raw executable (MIF file, no header)\n"
           "2nd: Input file (raw binary image)\n"
           "3rd: Output file\n"
           "4th: Project name or folder (optional)\n");
    return 0;
  }

  FILE *input, *output;
  unsigned char buffer[4];
  char tmp_string[1024];
  uint32_t tmp = 0, size = 0, checksum = 0;
  unsigned int i = 0;
  int operation = 0;
  unsigned long raw_exe_size = 0;

  if      (strcmp(argv[1], "-app_bin") == 0) { operation = OP_APP_BIN; }
  else if (strcmp(argv[1], "-app_vhd") == 0) { operation = OP_APP_VHD; }
  else if (strcmp(argv[1], "-bld_vhd") == 0) { operation = OP_BLD_VHD; }
  else if (strcmp(argv[1], "-raw_hex") == 0) { operation = OP_RAW_HEX; }
  else if (strcmp(argv[1], "-raw_bin") == 0) { operation = OP_RAW_BIN; }
  else if (strcmp(argv[1], "-raw_coe") == 0) { operation = OP_RAW_COE; }
  else if (strcmp(argv[1], "-raw_mem") == 0) { operation = OP_RAW_MEM; }
  else if (strcmp(argv[1], "-raw_mif") == 0) { operation = OP_RAW_MIF; }
  else {
    printf("Invalid operation '%s'!\n", argv[1]);
    return -1;
  }

  // open input file
  input = fopen(argv[2], "rb");
  if(input == NULL) {
    printf("Input file error (%s)!\n", argv[2]);
    return -2;
  }

  // get input file size
  fseek(input, 0L, SEEK_END);
  unsigned int input_size = (unsigned int)ftell(input);
  unsigned int input_words = input_size / 4;
  rewind(input);

  if ((input_size % 4) != 0) {
    printf("WARNING - image size is not a multiple of 4 bytes!\n");
  }

  // input file empty?
  if(input_size == 0) {
    printf("Input file is empty (%s)!\n", argv[2]);
    fclose(input);
    return -3;
  }

  // open output file
  output = fopen(argv[3], "wb");
  if(output == NULL) {
    printf("Output file error (%s)!\n", argv[3]);
    fclose(input);
    return -4;
  }


  // --------------------------------------------------------------------------
  // Image's compilation date and time
  // --------------------------------------------------------------------------
  time_t time_current;
  time(&time_current);
  struct tm *time_local = localtime(&time_current);
  char compile_time[64];

  snprintf(compile_time, 64, "%02d.%02d.%d %02d:%02d:%02d",
    time_local->tm_mday,
    time_local->tm_mon + 1,
    time_local->tm_year + 1900,
    time_local->tm_hour,
    time_local->tm_min,
    time_local->tm_sec
  );


  // --------------------------------------------------------------------------
  // Size of application (in bytes)
  // --------------------------------------------------------------------------
  fseek(input, 0L, SEEK_END);

  // get file size (raw executable)
  raw_exe_size = (unsigned long)ftell(input);

  // go back to beginning
  rewind(input);


  // --------------------------------------------------------------------------
  // Generate BINARY executable for bootloader upload (with header)
  // --------------------------------------------------------------------------
  if (operation == OP_APP_BIN) {

    // reserve header space for signature
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);

    // reserve header space for size
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);

    // reserve header space for checksum
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);
    fputc(0, output);

    checksum = 0;
    size = 0;
    rewind(input);
    while(fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      checksum += tmp; // checksum: sum complement
      fputc(buffer[0], output);
      fputc(buffer[1], output);
      fputc(buffer[2], output);
      fputc(buffer[3], output);
      size += 4;
    }

    rewind(output);
    // header: signature
    fputc((unsigned char)((signature >>  0) & 0xFF), output);
    fputc((unsigned char)((signature >>  8) & 0xFF), output);
    fputc((unsigned char)((signature >> 16) & 0xFF), output);
    fputc((unsigned char)((signature >> 24) & 0xFF), output);
    // header: size
    fputc((unsigned char)((size >>  0) & 0xFF), output);
    fputc((unsigned char)((size >>  8) & 0xFF), output);
    fputc((unsigned char)((size >> 16) & 0xFF), output);
    fputc((unsigned char)((size >> 24) & 0xFF), output);
    // header: checksum (sum complement)
    checksum = (~checksum) + 1;
    fputc((unsigned char)((checksum >>  0) & 0xFF), output);
    fputc((unsigned char)((checksum >>  8) & 0xFF), output);
    fputc((unsigned char)((checksum >> 16) & 0xFF), output);
    fputc((unsigned char)((checksum >> 24) & 0xFF), output);
  }


  // --------------------------------------------------------------------------
  // Generate APPLICATION executable memory initialization image package (IMEM)
  // --------------------------------------------------------------------------
  else if (operation == OP_APP_VHD) {

    // header
    snprintf(tmp_string, sizeof(tmp_string),
      "-- The NEORV32 RISC-V Processor - github.com/stnolting/neorv32\n"
      "-- Auto-generated memory initialization image (for internal IMEM)\n"
      "-- Source: %s/%s\n"
      "-- Built: %s\n"
      "\n"
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "library neorv32;\n"
      "use neorv32.neorv32_package.all;\n"
      "\n"
      "package neorv32_application_image is\n"
      "\n"
      "constant application_init_size_c  : natural := %lu; -- bytes\n"
      "constant application_init_image_c : mem32_t := (\n",
      argv[4], argv[2], compile_time, raw_exe_size);
    fputs(tmp_string, output);

    i = 0;
    while (i < (input_words-1)) {
      if (fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
        tmp  = (uint32_t)(buffer[0] << 0);
        tmp |= (uint32_t)(buffer[1] << 8);
        tmp |= (uint32_t)(buffer[2] << 16);
        tmp |= (uint32_t)(buffer[3] << 24);
        snprintf(tmp_string, sizeof(tmp_string), "x\"%08x\",\n", (unsigned int)tmp);
        fputs(tmp_string, output);
        i++;
      }
      else {
        printf("Unexpected input file end!\n");
        break;
      }
    }

    if (fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      snprintf(tmp_string, sizeof(tmp_string), "x\"%08x\"\n", (unsigned int)tmp);
      fputs(tmp_string, output);
      i++;
    }
    else {
      printf("Unexpected input file end!\n");
    }

    // end
    snprintf(tmp_string, sizeof(tmp_string),
      ");\n"
      "\n"
      "end neorv32_application_image;\n");
    fputs(tmp_string, output);
  }


  // --------------------------------------------------------------------------
  // Generate BOOTLOADER executable memory initialization image package (BOOTROM)
  // --------------------------------------------------------------------------
  else if (operation == OP_BLD_VHD) {

    // header
    snprintf(tmp_string, sizeof(tmp_string),
      "-- The NEORV32 RISC-V Processor - github.com/stnolting/neorv32\n"
      "-- Auto-generated memory initialization image (for internal BOOTROM)\n"
      "-- Source: %s/%s\n"
      "-- Built: %s\n"
      "\n"
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "library neorv32;\n"
      "use neorv32.neorv32_package.all;\n"
      "\n"
      "package neorv32_bootloader_image is\n"
      "\n"
      "constant bootloader_init_size_c  : natural := %lu; -- bytes\n"
      "constant bootloader_init_image_c : mem32_t := (\n",
      argv[4], argv[2], compile_time, raw_exe_size);
    fputs(tmp_string, output);

    i = 0;
    while (i < (input_words-1)) {
      if (fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
        tmp  = (uint32_t)(buffer[0] << 0);
        tmp |= (uint32_t)(buffer[1] << 8);
        tmp |= (uint32_t)(buffer[2] << 16);
        tmp |= (uint32_t)(buffer[3] << 24);
        snprintf(tmp_string, sizeof(tmp_string), "x\"%08x\",\n", (unsigned int)tmp);
        fputs(tmp_string, output);
        i++;
      }
      else {
        printf("Unexpected input file end!\n");
        break;
      }
    }

    if (fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      snprintf(tmp_string, sizeof(tmp_string), "x\"%08x\"\n", (unsigned int)tmp);
      fputs(tmp_string, output);
      i++;
    }
    else {
      printf("Unexpected input file end!\n");
    }

    // end
    snprintf(tmp_string, sizeof(tmp_string),
      ");\n"
      "\n"
      "end neorv32_bootloader_image;\n");
    fputs(tmp_string, output);
  }


  // --------------------------------------------------------------------------
  // Generate RAW APPLICATION's executable ASCII hex file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_HEX) {

    while(fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      snprintf(tmp_string, sizeof(tmp_string), "%08x\n", (unsigned int)tmp);
      fputs(tmp_string, output);
    }
  }


  // --------------------------------------------------------------------------
  // Generate RAW APPLICATION's executable binary file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_BIN) {

    while(fread(&buffer, sizeof(unsigned char), 1, input) != 0) {
      fputc(buffer[0], output);
    }
  }


  // --------------------------------------------------------------------------
  // Generate RAW APPLICATION's executable COE file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_COE) {

    // header
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_radix=16;\n");
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_vector=\n");
    fputs(tmp_string, output);

    i = 0;
    while(fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      if (i == (input_words-1)) {
        snprintf(tmp_string, sizeof(tmp_string), "%08x;\n", (unsigned int)tmp);
      }
      else {
        snprintf(tmp_string, sizeof(tmp_string), "%08x,\n", (unsigned int)tmp);
      }
      fputs(tmp_string, output);
      i++;
    }
  }


  // --------------------------------------------------------------------------
  // Generate RAW APPLICATION's executable MEM file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_MEM) {

    i = 0;
    while(fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      snprintf(tmp_string, sizeof(tmp_string), "@%08x %08x\n", (unsigned int)i, (unsigned int)tmp);
      fputs(tmp_string, output);
      i++;
    }
  }


  // --------------------------------------------------------------------------
  // Generate RAW APPLICATION's executable MIF file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_MIF) {

    // header
    snprintf(tmp_string, sizeof(tmp_string), "DEPTH = %lu;\n", raw_exe_size/4); // memory depth in words
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "WIDTH = 32;\n"); // bits per data word
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "ADDRESS_RADIX = HEX;\n"); // hexadecimal address format
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "DATA_RADIX = HEX;\n"); // hexadecimal data format
    fputs(tmp_string, output);

    snprintf(tmp_string, sizeof(tmp_string), "CONTENT\n");
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "BEGIN\n");
    fputs(tmp_string, output);
    i = 0;
    while(fread(&buffer, sizeof(unsigned char), 4, input) != 0) {
      tmp  = (uint32_t)(buffer[0] << 0);
      tmp |= (uint32_t)(buffer[1] << 8);
      tmp |= (uint32_t)(buffer[2] << 16);
      tmp |= (uint32_t)(buffer[3] << 24);
      snprintf(tmp_string, sizeof(tmp_string), "%08x : %08x;\n", (unsigned int)i, (unsigned int)tmp);
      fputs(tmp_string, output);
      i++;
    }

    // footer
    snprintf(tmp_string, sizeof(tmp_string), "END;\n");
    fputs(tmp_string, output);
  }


  // --------------------------------------------------------------------------
  // Invalid operation
  // --------------------------------------------------------------------------
  else {
    printf("Invalid operation!\n");
    fclose(input);
    fclose(output);
    return -1;
  }


  // --------------------------------------------------------------------------
  // Clean up
  // --------------------------------------------------------------------------
  fclose(input);
  fclose(output);

  return 0;
}
