# NEORV32 Test Setup for the Digilent Nexys A7 and Nexys 4 DDR FPGA Boards

This setup provides a very simple script-based "demo setup" that allows to check out the NEORV32 processor on the Digilent Nexys A7 and Nexys 4 DDR boards.
It uses the simplified [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity, which is a wrapper for the actual processor
top entity that provides a minimalistic interface (clock, reset, UART and 4 LEDs).

* FPGA Boards:
  * :books: [Digilent Nexys A7 FPGA Boards](https://reference.digilentinc.com/reference/programmable-logic/nexys-a7/reference-manual)
  * :books: [Digilent Nexys 4 DDR FPGA Board](https://reference.digilentinc.com/reference/programmable-logic/nexys-4-ddr/reference-manual)
* FPGAs:
  * Xilinx Artix-7 `XC7A50TCSG324-1`
  * Xilinx Artix-7 `XC7A100TCSG324-1`
* Toolchain: Xilinx Vivado (tested with Vivado 2020.2)


## NEORV32 Configuration

:information_source: See the top entity [`rtl/test_setups/neorv32_test_setup_bootloader.vhd` ](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) for
configuration and entity details and [`nexys_a7_test_setup.xdc`](https://github.com/AWenzel83/neorv32/blob/nexys_a7_example/boards/nexys-a7-test-setup/nexys_a7_test_setup.xdc)
for the according FPGA pin mapping.

* CPU: `rv32imcu_Zicsr` + 4 `HPM` (hardware performance monitors)
* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (internal DMEM), bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `WDT`
* Tested with version [`1.5.3.3`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 100MHz from on-board oscillator
* Reset: Via dedicated on-board "RESET" button
* GPIO output port `gpio_o` bits 0..7 are connected to the green on-board LEDs (LD0 - LD7); LD0 is the bootloader status LED
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the on-board USB-UART chip
