// ================================================================================ //
// NEORV32 executable image generator                                               //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2026 Stephan Nolting. All rights reserved.                  //
// Licensed under the BSD-3-Clause license, see LICENSE for details.                //
// SPDX-License-Identifier: BSD-3-Clause                                            //
// ================================================================================ //

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// executable signature ("magic word")
const uint32_t signature = 0xB007C0DE;

// output file types (operation select)
enum operation_enum {
  OP_EXE,
  OP_VHD,
  OP_HEX,
  OP_BIN,
  OP_COE,
  OP_MEM,
  OP_MIF
};

void print_help(void){
  printf(
    "NEORV32 executable image generator\n"
    "\n"
    "Usage:    image_gen [options]\n"
    "Example:  image_gen -i main.bin -o main_exe.bin -t exe\n"
    "\n"
    "Options:\n"
    "  -h            Show this help text and exit\n"
    "  -i file_name  Input binary file name; mandatory\n"
    "  -o file_name  Output file name; mandatory\n"
    "  -t type       Type of image to generate; default is 'exe'\n"
    "\n"
    "Image type:\n"
    "  exe  Executable for bootloader upload (binary file with header) \n"
    "  vhd  VHDL memory image (raw executable)\n"
    "  hex  ASCII hex file (raw executable)\n"
    "  bin  Binary file (raw executable)\n"
    "  coe  COE file (raw executable)\n"
    "  mem  MEM file (raw executable)\n"
    "  mif  MIF file (raw executable)\n"
  );
}

int main(int argc, char *argv[]) {

  FILE *input = NULL, *output = NULL;
  char *input_file = NULL, *output_file = NULL, tmp_string[1024];
  uint32_t u32 = 0, checksum = 0;
  unsigned int i = 0, operation = OP_EXE, raw_exe_size = 0, ext_exe_size = 0;
  unsigned char byte = 0;

  // show help menu if there are no arguments
  if (argc <= 1) {
    print_help();
    return 0;
  }

  // parse arguments
  for (i = 1; i < argc; i++) {
    // show help
    if (strcmp(argv[i], "-h") == 0) {
      print_help();
      return 0;
    }
    // input file
    else if (strcmp(argv[i], "-i") == 0) {
      input_file = argv[++i];
    }
    // output file
    else if (strcmp(argv[i], "-o") == 0) {
      output_file = argv[++i];
    }
    // type
    else if (strcmp(argv[i], "-t") == 0) {
      i++;
      if      (strcmp(argv[i], "exe") == 0) { operation = OP_EXE; }
      else if (strcmp(argv[i], "vhd") == 0) { operation = OP_VHD; }
      else if (strcmp(argv[i], "hex") == 0) { operation = OP_HEX; }
      else if (strcmp(argv[i], "bin") == 0) { operation = OP_BIN; }
      else if (strcmp(argv[i], "coe") == 0) { operation = OP_COE; }
      else if (strcmp(argv[i], "mem") == 0) { operation = OP_MEM; }
      else if (strcmp(argv[i], "mif") == 0) { operation = OP_MIF; }
      else {
        printf("[ERROR] Invalid type '%s'!\n", argv[i]);
        return -1;
      }
    }
    // unknown
    else {
      printf("[ERROR] Unknown flag '%s'!\n", argv[i]);
      return -1;
    }
  }

  // open input file
  input = fopen(input_file, "rb");
  if(input == NULL) {
    printf("[ERROR] Input file error (%s)!\n", input_file);
    return -2;
  }

  // get input file size
  fseek(input, 0L, SEEK_END);
  raw_exe_size = (unsigned int)ftell(input);
  rewind(input);
  if (raw_exe_size == 0) {// input file empty?
    printf("[ERROR] Input file is empty (%s)!\n", input_file);
    fclose(input);
    return -2;
  }
  if ((raw_exe_size % 4) != 0) {
    printf("[WARNING] Input image size is not a multiple of 4 bytes!\n");
  }

  // make sure memory array is a power of two
  ext_exe_size = 4;
  while (ext_exe_size < raw_exe_size) {
    ext_exe_size *= 2;
  }

  // open output file
  output = fopen(output_file, "wb");
  if (output == NULL) {
    printf("[ERROR] Output file error (%s)!\n", output_file);
    fclose(output);
    return -2;
  }

  // --------------------------------------------------------------------------
  // executable for bootloader upload (including header)
  // --------------------------------------------------------------------------
  if (operation == OP_EXE) {

    // reserve header space
    for (i=0; i<12; i++) {
      fputc(0, output);
    }

    // actual data
    checksum = 0;
    rewind(input);
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      checksum += u32; // checksum: sum complement
      fputc((unsigned char)((u32 >>  0) & 0xFF), output);
      fputc((unsigned char)((u32 >>  8) & 0xFF), output);
      fputc((unsigned char)((u32 >> 16) & 0xFF), output);
      fputc((unsigned char)((u32 >> 24) & 0xFF), output);
    }

    rewind(output);
    // header: signature
    fputc((unsigned char)((signature >>  0) & 0xFF), output);
    fputc((unsigned char)((signature >>  8) & 0xFF), output);
    fputc((unsigned char)((signature >> 16) & 0xFF), output);
    fputc((unsigned char)((signature >> 24) & 0xFF), output);
    // header: size
    fputc((unsigned char)((raw_exe_size >>  0) & 0xFF), output);
    fputc((unsigned char)((raw_exe_size >>  8) & 0xFF), output);
    fputc((unsigned char)((raw_exe_size >> 16) & 0xFF), output);
    fputc((unsigned char)((raw_exe_size >> 24) & 0xFF), output);
    // header: checksum (sum complement)
    checksum = ~checksum;
    fputc((unsigned char)((checksum >>  0) & 0xFF), output);
    fputc((unsigned char)((checksum >>  8) & 0xFF), output);
    fputc((unsigned char)((checksum >> 16) & 0xFF), output);
    fputc((unsigned char)((checksum >> 24) & 0xFF), output);
  }

  // --------------------------------------------------------------------------
  // VHDL memory image (package name = output file name)
  // --------------------------------------------------------------------------
  else if (operation == OP_VHD) {

    // remove path from output file
    const char *filename = strrchr(output_file, '/'); // Linux
    if (filename == NULL) {
      filename = strrchr(output_file, '\\'); // maybe Windows?
    }
    if (filename) {
      filename++;
    }
    else {
      filename = output_file;
    }

    // remove suffix from output file
    char pkg_name[256];
    strncpy(pkg_name, filename, sizeof(pkg_name));
    pkg_name[sizeof(pkg_name)-1] = '\0';

    char *suffix = strrchr(pkg_name, '.');
    if (suffix) {
      *suffix = '\0';
    }

    // header
    snprintf(tmp_string, sizeof(tmp_string),
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "package %s is\n"
      "\n"
      "type rom_t is array (0 to %u) of std_ulogic_vector(31 downto 0);\n"
      "constant image_size_c : natural := %u;\n"
      "constant image_data_c : rom_t := (\n",
      pkg_name, (ext_exe_size/4)-1, raw_exe_size);
    fputs(tmp_string, output);

    for (i=0; i<(raw_exe_size/4); i++) {
      if (fread(&u32, sizeof(uint32_t), 1, input) != 0) {
        snprintf(tmp_string, sizeof(tmp_string), "x\"%08x\",\n", (unsigned int)u32);
        fputs(tmp_string, output);
      }
      else {
        printf("[WARNING] Unexpected input file end!\n");
        break;
      }
    }

    // end
    snprintf(tmp_string, sizeof(tmp_string),
      "others => (others => '0')\n"
      ");\n"
      "\n"
      "end %s;\n", pkg_name);
    fputs(tmp_string, output);
  }

  // --------------------------------------------------------------------------
  // executable ASCII hex file
  // --------------------------------------------------------------------------
  else if (operation == OP_HEX) {

    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      snprintf(tmp_string, sizeof(tmp_string), "%08x\n", (unsigned int)u32);
      fputs(tmp_string, output);
    }
  }

  // --------------------------------------------------------------------------
  // executable binary file
  // --------------------------------------------------------------------------
  else if (operation == OP_BIN) {

    while(fread(&byte, sizeof(unsigned char), 1, input) != 0) {
      fputc(byte, output);
    }
  }

  // --------------------------------------------------------------------------
  // executable COE file
  // --------------------------------------------------------------------------
  else if (operation == OP_COE) {

    // header
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_radix=16;\n");
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_vector=\n");
    fputs(tmp_string, output);

    i = 0;
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      if (i == ((raw_exe_size/4)-1)) {
        snprintf(tmp_string, sizeof(tmp_string), "%08x;\n", (unsigned int)u32);
      }
      else {
        snprintf(tmp_string, sizeof(tmp_string), "%08x,\n", (unsigned int)u32);
      }
      fputs(tmp_string, output);
      i++;
    }
  }

  // --------------------------------------------------------------------------
  // executable MEM file
  // --------------------------------------------------------------------------
  else if (operation == OP_MEM) {

    i = 0;
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      snprintf(tmp_string, sizeof(tmp_string), "@%08x %08x\n", (unsigned int)i, (unsigned int)u32);
      fputs(tmp_string, output);
      i++;
    }
  }

  // --------------------------------------------------------------------------
  // executable MIF file
  // --------------------------------------------------------------------------
  else if (operation == OP_MIF) {

    // header
    snprintf(tmp_string, sizeof(tmp_string), "DEPTH = %u;\n", raw_exe_size/4); // memory depth in words
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
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      snprintf(tmp_string, sizeof(tmp_string), "%08x : %08x;\n", (unsigned int)i, (unsigned int)u32);
      fputs(tmp_string, output);
      i++;
    }

    // footer
    snprintf(tmp_string, sizeof(tmp_string), "END;\n");
    fputs(tmp_string, output);
  }

  // invalid operation
  else {
    printf("[ERROR] Invalid operation!\n");
    fclose(input);
    fclose(output);
    return -1;
  }

  // clean up
  fclose(input);
  fclose(output);

  return 0;
}
