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

// executable signature identifier ("magic word", for bootloader only)
const uint32_t signature_c = 0x214F454E;

// output image types (operation select)
enum operation_enum {
  OP_EXE,
  OP_VHD,
  OP_BIN,
  OP_COE,
  OP_MEM,
  OP_MIF
};

// bootloader executable header
typedef struct __attribute__((packed,aligned(4))) {
  uint32_t signature;
  uint32_t base_addr;
  uint32_t size;
  uint32_t checksum;
} exe_header_t;

// ************************************************************
// Write 32-bit data to file (little-Endian).
// ************************************************************
void write32(uint32_t d, FILE *f) {

  fputc((unsigned char)((d >>  0) & 0xFF), f);
  fputc((unsigned char)((d >>  8) & 0xFF), f);
  fputc((unsigned char)((d >> 16) & 0xFF), f);
  fputc((unsigned char)((d >> 24) & 0xFF), f);
}

// ************************************************************
// Show help menu.
// ************************************************************
void print_help(void){

  printf(
    "NEORV32 executable image generator\n"
    "\n"
    "Usage:    image_gen [options]\n"
    "Example:  image_gen -i elf.bin -o main_exe.bin -b A0000000 -t exe\n"
    "\n"
    "Options:\n"
    "  -h            Show this help text and exit\n"
    "  -i file_name  Flattened binary input file\n"
    "  -o file_name  Image output file\n"
    "  -t format     Image output format\n"
    "  -b address    Bootloader exe relocation address (hex; for \"-t exe\" only)\n"
    "\n"
    "Image formats (using little-Endian byte ordering):\n"
    "  exe  Executable for bootloader upload (binary file with header) \n"
    "  vhd  VHDL memory image (raw executable)\n"
    "  bin  Binary file (raw executable)\n"
    "  coe  COE file (8x hex per line, ASCII, raw executable)\n"
    "  mem  MEM file (8x hex per line, ASCII, raw executable)\n"
    "  mif  MIF file (8x hex per line, ASCII, raw executable)\n"
  );
}

// ************************************************************
// Main.
// ************************************************************
int main(int argc, char *argv[]) {

  FILE *input = NULL, *output = NULL;
  char *input_file = NULL, *output_file = NULL;
  uint32_t checksum = 0, base_addr = 0xFFFFFFFF;
  int i = 0;
  unsigned int operation = OP_EXE, raw_exe_size = 0, ext_exe_size = 0;

  // show help menu if there are no arguments
  if (argc <= 1) {
    print_help();
    return 0;
  }

  // --------------------------------------------------------------------------
  // parse arguments
  // --------------------------------------------------------------------------

  for (i = 1; i < argc; i++) {
    // show help
    if (strcmp(argv[i], "-h") == 0) {
      print_help();
      return 0;
    }
    // input file
    else if (strcmp(argv[i], "-i") == 0) {
      i++;
      if (i >= (unsigned int)argc) {
        printf("[ERROR] Missing argument for '-i'!\n");
        return -1;
      }
      input_file = argv[i];
    }
    // output file
    else if (strcmp(argv[i], "-o") == 0) {
      i++;
      if (i >= (unsigned int)argc) {
        printf("[ERROR] Missing argument for '-o'!\n");
        return -1;
      }
      output_file = argv[i];
    }
    // type
    else if (strcmp(argv[i], "-t") == 0) {
      i++;
      if (i >= (unsigned int)argc) {
        printf("[ERROR] Missing argument for '-t'!\n");
        return -1;
      }
      if      (strcmp(argv[i], "exe") == 0) { operation = OP_EXE; }
      else if (strcmp(argv[i], "vhd") == 0) { operation = OP_VHD; }
      else if (strcmp(argv[i], "bin") == 0) { operation = OP_BIN; }
      else if (strcmp(argv[i], "coe") == 0) { operation = OP_COE; }
      else if (strcmp(argv[i], "mem") == 0) { operation = OP_MEM; }
      else if (strcmp(argv[i], "mif") == 0) { operation = OP_MIF; }
      else {
        printf("[ERROR] Invalid type '%s'!\n", argv[i]);
        return -1;
      }
    }
    // bootloader relocation/base address
    else if (strcmp(argv[i], "-b") == 0) {
      i++;
      if (i >= (unsigned int)argc) {
        printf("[ERROR] Missing argument for '-b'!\n");
        return -1;
      }
      base_addr = (uint32_t)strtoul(argv[i], NULL, 0);
    }
    // invalid
    else {
      printf("[ERROR] Invalid flag '%s'!\n", argv[i]);
      return -1;
    }
  }

  // we need a base address for the bootloader executable
  if ((operation == OP_EXE) && (base_addr == -1)) {
    printf("[ERROR] Missing '-b' argument!\n");
    return -1;
  }

  // --------------------------------------------------------------------------
  // open input/output files
  // ****************************************
  input = fopen(input_file, "rb");
  if (input == NULL) {
    printf("[ERROR] Input file error (%s)!\n", input_file);
    return -2;
  }

  output = fopen(output_file, "wb");
  if (output == NULL) {
    printf("[ERROR] Output file error (%s)!\n", output_file);
    fclose(input);
    return -2;
  }

  // --------------------------------------------------------------------------
  // read flat binary input
  // --------------------------------------------------------------------------

  fseek(input, 0, SEEK_END);
  long flat_bin_size = ftell(input);
  rewind(input);

  // binary size
  if (flat_bin_size <= 0) {
    printf("[ERROR] Input file is empty (%s)!\n", input_file);
    fclose(input);
    fclose(output);
    return -2;
  }
  raw_exe_size = (unsigned int)flat_bin_size;

  // base address (= entry point)
  uint32_t base_addr = (uint32_t)elf.e_entry;

  uint8_t *raw_image = calloc(padded_size, 1); // zero-padded
  if (!raw_image) {
    printf("[ERROR] malloc failed!\n");
    fclose(input);
    fclose(output);
    return -1;
  }

  if (fread(raw_image, 1, raw_exe_size, input) != raw_exe_size) {
    printf("[ERROR] Failed to read input file (%s)!\n", input_file);
    free(raw_image);
    fclose(input);
    fclose(output);
    return -2;
  }
  fclose(input);

  // ****************************************
  // generate raw image
  // ****************************************

  // debug
//printf(".text:   %d bytes\n", text_size);
//printf(".rodata: %d bytes\n", rodata_size);
//printf(".data:   %d bytes\n", data_size);

  // final image size
  raw_exe_size = text_size + rodata_size + data_size;
  if (raw_exe_size == 0) {// input file empty?
    printf("[ERROR] Image is empty!\n");
    return -2;
  }
  if ((raw_exe_size % 4) != 0) {
    printf("[WARNING] Image size is not a multiple of 4 bytes!\n");
  }

  // make sure memory array is a power of two
  ext_exe_size = 4;
  while (ext_exe_size < raw_exe_size) {
    ext_exe_size *= 2;
  }

  // --------------------------------------------------------------------------
  // executable for bootloader upload (including header)
  // --------------------------------------------------------------------------

  if (operation == OP_EXE) {

    exe_header_t header;

    // reserve header space
    for (i = 0; i < sizeof(header); i++) {
      fputc(0, output);
    }

    // actual data and checksum
    checksum = 0;
    for (i = 0; i < raw_exe_size/4; i++) {
      checksum += raw_image32[i];
      write32(raw_image32[i], output);
    }

    // setup header
    rewind(output);
    header.signature = signature_c;
    header.base_addr = base_addr;
    header.size      = raw_exe_size;
    header.checksum  = ~checksum;
    write32(header.signature, output);
    write32(header.base_addr, output);
    write32(header.size, output);
    write32(header.checksum, output);

    // report
    printf("Executable (EXE): %u bytes @ 0x%08X, checksum = 0x%08X\n",
           (unsigned int)header.size, (unsigned int)header.base_addr, (unsigned int)header.checksum);
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
    fprintf(output,
      "library ieee;\n"
      "use ieee.std_logic_1164.all;\n"
      "\n"
      "package %s is\n"
      "\n"
      "type rom_t is array (0 to %u) of std_ulogic_vector(31 downto 0);\n"
      "constant image_size_c : natural := %u;\n"
      "constant image_data_c : rom_t := (\n",
      pkg_name, (ext_exe_size/4)-1, raw_exe_size);

    // data
    for (i = 0; i < raw_exe_size/4; i++) {
      fprintf(output, "x\"%08x\",\n", (unsigned int)raw_image32[i]);
    }

    // end
    fprintf(output,
      "others => (others => '0')\n"
      ");\n"
      "\n"
      "end %s;\n", pkg_name);

    // report
    printf("Executable (VHD): %u bytes\n", raw_exe_size);
  }

  // --------------------------------------------------------------------------
  // executable plain-binary file
  // --------------------------------------------------------------------------

  else if (operation == OP_BIN) {

    for (i = 0; i < raw_exe_size; i++) {
      fputc((unsigned char)(raw_image[i]), output);
    }

    // report
    printf("Executable (BIN): %u bytes\n", raw_exe_size);
  }

  // --------------------------------------------------------------------------
  // executable COE file
  // --------------------------------------------------------------------------

  else if (operation == OP_COE) {

    // header
    fputs("memory_initialization_radix=16;\n", output);
    fputs("memory_initialization_vector=\n", output);

    for (i = 0; i < raw_exe_size/4; i++) {
      if (i == ((raw_exe_size/4)-1)) {
        fprintf(output, "%08x;\n", (unsigned int)raw_image32[i]);
      }
      else {
        fprintf(output, "%08x,\n", (unsigned int)raw_image32[i]);
      }
    }

    // report
    printf("Executable (COE): %u bytes\n", raw_exe_size);
  }

  // --------------------------------------------------------------------------
  // executable MEM file
  // --------------------------------------------------------------------------

  else if (operation == OP_MEM) {

    for (i = 0; i < raw_exe_size/4; i++) {
      fprintf(output, "@%08x %08x\n", (unsigned int)i, (unsigned int)raw_image32[i]);
    }

    // report
    printf("Executable (MEM): %u bytes\n", raw_exe_size);
  }

  // --------------------------------------------------------------------------
  // executable MIF file
  // --------------------------------------------------------------------------

  else if (operation == OP_MIF) {

    // header
    fprintf(output,
      "DEPTH = %u;\n"
      "WIDTH = 32;\n"
      "ADDRESS_RADIX = HEX;\n"
      "DATA_RADIX = HEX;\n"
      "CONTENT\n"
      "BEGIN\n",
      raw_exe_size/4
    );

    // data
    for (i = 0; i < raw_exe_size/4; i++) {
      fprintf(output, "%08x : %08x;\n", (unsigned int)i, (unsigned int)raw_image32[i]);
    }

    // footer
    fputs("END;\n", output);

    // report
    printf("Executable (MIF): %u bytes\n", raw_exe_size);
  }

  // invalid operation
  else {
    printf("[ERROR] Invalid operation!\n");
    free(raw_image);
    fclose(output);
    return -1;
  }

  // clean up
  free(raw_image);
  fclose(output);

  return 0;
}
