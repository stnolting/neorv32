# Running the RISC-V Compliance Framework in Simulation


The following tasks are executed by the device makefiles:

* replace the original processor's IMEM rtl file by a simulation-optimized IMEM (ROM!)
* `sed` command is used to modify the default testbench (`neorv32/sim/neorv32_tb.vhd`):
  * enable/disable the required `CPU_EXTENSION_RISCV_xxx` VHDL configuration generics in the testbench (`neorv32/sim/neorv32_tb.vhd`)
  * configure the processor memory configuration (use internal IMEM, IMEM as ROM, IMEM size of 2MB)
* compile test code and install application image to processor's `rtl/core` folder
  * compilation uses the `link.imem_rom.ld` linker script as default; code (the test code) is executed from IMEM (which is read-only); data including signatureis stored to DMEM (RAM)
  * certain areas in the DMEM are initialized using port code in `model_test.h` (`RVTEST` = 0xbabecafe and `SIGNATURE` = 0xdeadbeef); can be disabled using `RISCV_TARGET_FLAGS=-DNEORV32_NO_DATA_INIT`
* `sed` command is used to modify the default application image that is generated during compilation (`neorv32/rtl/core/neorv32_application_image.vhd`):
  * the array size of the application image is set to 2MB
* the processor is simulated using the default testbench using GHDL
* the results are dumped via the SIM_MODE feature of the UART
  * the according code can be found in the `RVMODEL_HALT` macro in `model_test.h`
  * data output (the "signature") is zero-padded to be always a multiple of 16 bytes

:warning: The `Zifencei` test requires the r/w/e capabilities of the original IMEM rtl file.
Hence, the original file is restored for this test. Also, this test uses `link.imem_ram.ld` as linker script since the
IMEM is used as RAM to allow self-modifying code.
