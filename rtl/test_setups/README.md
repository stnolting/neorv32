# Test Setups

This folder contains very simple test setups that are intended for project beginners
to setup a minimal NEORV32 SoC. These setups are used in the :books:
[NEORV32 User Guide](https://stnolting.github.io/neorv32/ug/).
Note that these setups provides a minimalistic configuration to keep
things at a simple level at first. Additional CPU ISA extensions, performance options and
optional peripheral modules can be enabled by specifying the according :book:
[configuration generics](https://stnolting.github.io/neorv32/#_processor_top_entity_generics).


### Setup Top Entities

#### Clocking and Reset

All test setups require an external clock (via `clk_i` signal) and an external
low-active reset (via `rstn_i` signal).

#### Configuration Generics

Each setup provides three elementary generics that can/should be adapted to fit
your FPGA/board.

* The clock speed in Hz **has to be specified** via the `CLOCK_SPEED` generic to fit your clock source.
* The processor-internal instruction memory (IMEM) size _can be modified_ via the `IMEM_SIZE` generic.
* The processor-internal data memory (DMEM) size _can be modified_ via the `DMEM_SIZE` generic.

> [!WARNING]
> Modifying the memory sizes might require adaption of the NEORV32 linker script.


### > [`neorv32_test_setup_approm.vhd`](https://github.com/stnolting/neorv32/blob/main/rtl/test_setups/neorv32_test_setup_approm.vhd)

This setup configures a `rv32imc_Zicsr_Zicntr` CPU with 16kB IMEM (as pre-initialized ROM),
8kB DMEM and includes the **GPIO** module to drive 8 external signals (`gpio_o`)
and the **CLINT** module for generating timer interrupts.
The setup uses the ["direct boot"](https://stnolting.github.io/neorv32/#_direct_boot)
configuration, so software applications are "installed" directly into the
processor-internal IMEM (via the bitstream) during synthesis.

> [!TIP]
> See User Guide section [Installing an Executable Directly Into Memory](https://stnolting.github.io/neorv32/ug/#_installing_an_executable_directly_into_memory).


### > [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/main/rtl/test_setups/neorv32_test_setup_bootloader.vhd)

This setup configures a `rv32imc_Zicsr_Zicntr` CPU with 16kB IMEM (as RAM), 8kB DMEM
and includes the **GPIO** module to drive 8 external signals (`gpio_o`), the **CLINT**
module for generating timer interrupts and **UART0** to interface with the bootloader or application
(via `uart0_txd_o` and `uart0_rxd_i`) via a serial terminal.
The setup uses the ["indirect boot"](https://stnolting.github.io/neorv32/#_indirect_boot)
configuration, so software applications can be uploaded and run at any timer via the bootloader
and a serial terminal.

> [!TIP]
> See User Guide section [Uploading and Starting of a Binary Executable Image via UART](https://stnolting.github.io/neorv32/ug/#_uploading_and_starting_of_a_binary_executable_image_via_uart).


### > [`neorv32_test_setup_on_chip_debugger.vhd`](https://github.com/stnolting/neorv32/blob/main/rtl/test_setups/neorv32_test_setup_on_chip_debugger.vhd)

This setup configures a `rv32imc_Zicsr_Zicntr_Zifencei` CPU with 16kB IMEM (as RAM), 8kB DMEM
and includes the **GPIO** module to drive 8 external signals (`gpio_o`), the **CLINT**
module for generating timer interrupts, **UART0** to interface with the bootloader or application
(via `uart0_txd_o` and `uart0_rxd_i`) via a serial terminal and also the RISC-V-compatible
on-chip debugger (**OCD**), which is accessible via a standard JTAG interface (`jtag_*`).
The setup uses the ["indirect boot"](https://stnolting.github.io/neorv32/#_indirect_boot)
configuration, so software applications can be uploaded and run at any timer via the bootloader
and a serial terminal and also via the on-chip debugger.

> [!TIP]
> See User Guide section [Debugging using the On-Chip Debugger](https://stnolting.github.io/neorv32/ug/#_debugging_using_the_on_chip_debugger).
