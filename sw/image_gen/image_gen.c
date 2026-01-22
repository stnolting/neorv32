// ================================================================================ //
// NEORV32 executable image generator                                               //
// -------------------------------------------------------------------------------- //
// The NEORV32 RISC-V Processor - https://github.com/stnolting/neorv32              //
// Copyright (c) NEORV32 contributors.                                              //
// Copyright (c) 2020 - 2025 Stephan Nolting. All rights reserved.                  //
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
  OP_APP_BIN,
  OP_APP_VHD,
  OP_BLD_VHD,
  OP_RAW_HEX,
  OP_RAW_BIN,
  OP_RAW_COE,
  OP_RAW_MEM,
  OP_RAW_MIF
};

void print_help(void){
  printf(
    "NEORV32 executable image generator\n"
    "\n"
    "Usage:    image_gen [options]\n"
    "Example:  image_gen -i main.bin -o main_exe.bin -t app_bin\n"
    "\n"
    "Options:\n"
    "  -h             Show this help text and exit\n"
    "  -i file_name   Input binary file name; mandatory\n"
    "  -o file_name   Output file name; mandatory\n"
    "  -t type        Type of image to generate; default is 'app_bin'\n"
    "\n"
    "Image type:\n"
    "  app_bin   Application executable for bootloader upload (binary file with header) \n"
    "  app_vhd   Application memory image (IMEM VHDL package file)\n"
    "  bld_vhd   Bootloader memory image (BOOTROM VHDL package file)\n"
    "  raw_hex   ASCII hex file (raw executable)\n"
    "  raw_bin   Binary file (raw executable)\n"
    "  raw_coe   COE file (raw executable)\n"
    "  raw_mem   MEM file (raw executable)\n"
    "  raw_mif   MIF file (raw executable)\n"
  );
}

int main(int argc, char *argv[]) {

  FILE *input = NULL, *output = NULL;
  char *input_file = NULL, *output_file = NULL, tmp_string[1024];
  uint32_t u32 = 0, checksum = 0;
  unsigned int i = 0, operation = OP_APP_BIN, raw_exe_size = 0, ext_exe_size = 0, input_size = 0;
  unsigned char byte = 0;

  // show help menu if there are no arguments
  if (argc <= 1) {
    print_help();
    return 0;
  }

  // parse arguments
  for (int i = 1; i < argc; i++) {
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
      if      (strcmp(argv[i], "app_bin") == 0) { operation = OP_APP_BIN; }
      else if (strcmp(argv[i], "app_vhd") == 0) { operation = OP_APP_VHD; }
      else if (strcmp(argv[i], "bld_vhd") == 0) { operation = OP_BLD_VHD; }
      else if (strcmp(argv[i], "raw_hex") == 0) { operation = OP_RAW_HEX; }
      else if (strcmp(argv[i], "raw_bin") == 0) { operation = OP_RAW_BIN; }
      else if (strcmp(argv[i], "raw_coe") == 0) { operation = OP_RAW_COE; }
      else if (strcmp(argv[i], "raw_mem") == 0) { operation = OP_RAW_MEM; }
      else if (strcmp(argv[i], "raw_mif") == 0) { operation = OP_RAW_MIF; }
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
  input_size = (unsigned int)ftell(input);
  rewind(input);
  if (input_size == 0) {// input file empty?
    printf("[ERROR] Input file is empty (%s)!\n", input_file);
    fclose(input);
    return -2;
  }
  if ((input_size % 4) != 0) {
    printf("[WARNING] Input image size is not a multiple of 4 bytes!\n");
  }

  // open output file
  output = fopen(output_file, "wb");
  if (output == NULL) {
    printf("[ERROR] Output file error (%s)!\n", output_file);
    fclose(output);
    return -2;
  }


  // --------------------------------------------------------------------------
  // Application image size and memory array size
  // --------------------------------------------------------------------------
  fseek(input, 0L, SEEK_END);

  // get file size (raw executable)
  raw_exe_size = (unsigned int)ftell(input);

  // make sure memory array is a power of two
  ext_exe_size = 4;
  while (ext_exe_size < raw_exe_size) {
    ext_exe_size *= 2;
  }

  // back to beginning
  rewind(input);


  // --------------------------------------------------------------------------
  // Generate application executable for bootloader upload (including header)
  // --------------------------------------------------------------------------
  if (operation == OP_APP_BIN) {

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
    fputc((unsigned char)((input_size >>  0) & 0xFF), output);
    fputc((unsigned char)((input_size >>  8) & 0xFF), output);
    fputc((unsigned char)((input_size >> 16) & 0xFF), output);
    fputc((unsigned char)((input_size >> 24) & 0xFF), output);
    // header: checksum (sum complement)
    checksum = ~checksum;
    fputc((unsigned char)((checksum >>  0) & 0xFF), output);
    fputc((unsigned char)((checksum >>  8) & 0xFF), output);
    fputc((unsigned char)((checksum >> 16) & 0xFF), output);
    fputc((unsigned char)((checksum >> 24) & 0xFF), output);
  }


  // --------------------------------------------------------------------------
  // Generate application executable memory initialization image package
  // --------------------------------------------------------------------------
  else if (operation == OP_APP_VHD) {

    // header
    snprintf(tmp_string, sizeof(tmp_string),
      "-- The NEORV32 RISC-V Processor\n"
      "-- Auto-generated memory image for internal IMEM\n"
      "\n"
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "package neorv32_application_image is\n"
      "\n"
      "constant application_image_size_c : natural := %u;\n"
      "type rom_t is array (0 to %u) of std_ulogic_vector(31 downto 0);\n"
      "constant application_image_data_c : rom_t := (\n",
      raw_exe_size, (ext_exe_size/4)-1);
    fputs(tmp_string, output);

    for (i=0; i<(input_size/4); i++) {
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
      "end neorv32_application_image;\n");
    fputs(tmp_string, output);
  }


  // --------------------------------------------------------------------------
  // Generate bootloader executable memory initialization image package
  // --------------------------------------------------------------------------
  else if (operation == OP_BLD_VHD) {

    // header
    snprintf(tmp_string, sizeof(tmp_string),
      "-- The NEORV32 RISC-V Processor\n"
      "-- Auto-generated memory image for internal BOOTROM\n"
      "\n"
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "package neorv32_bootloader_image is\n"
      "\n"
      "constant bootloader_image_size_c : natural := %u;\n"
      "type rom_t is array (0 to %u) of std_ulogic_vector(31 downto 0);\n"
      "constant bootloader_image_data_c : rom_t := (\n",
      raw_exe_size, (ext_exe_size/4)-1);
    fputs(tmp_string, output);

    for (i=0; i<(input_size/4); i++) {
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
      "end neorv32_bootloader_image;\n");
    fputs(tmp_string, output);
  }


  // --------------------------------------------------------------------------
  // Generate application executable ASCII hex file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_HEX) {

    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      snprintf(tmp_string, sizeof(tmp_string), "%08x\n", (unsigned int)u32);
      fputs(tmp_string, output);
    }
  }


  // --------------------------------------------------------------------------
  // Generate application executable binary file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_BIN) {

    while(fread(&byte, sizeof(unsigned char), 1, input) != 0) {
      fputc(byte, output);
    }
  }


  // --------------------------------------------------------------------------
  // Generate application executable COE file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_COE) {

    // header
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_radix=16;\n");
    fputs(tmp_string, output);
    snprintf(tmp_string, sizeof(tmp_string), "memory_initialization_vector=\n");
    fputs(tmp_string, output);

    i = 0;
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      if (i == ((input_size/4)-1)) {
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
  // Generate application executable MEM file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_MEM) {

    i = 0;
    while(fread(&u32, sizeof(uint32_t), 1, input) != 0) {
      snprintf(tmp_string, sizeof(tmp_string), "@%08x %08x\n", (unsigned int)i, (unsigned int)u32);
      fputs(tmp_string, output);
      i++;
    }
  }


  // --------------------------------------------------------------------------
  // Generate application executable MIF file
  // --------------------------------------------------------------------------
  else if (operation == OP_RAW_MIF) {

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


  // --------------------------------------------------------------------------
  // Invalid operation
  // --------------------------------------------------------------------------
  else {
    printf("[ERROR] Invalid operation!\n");
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
