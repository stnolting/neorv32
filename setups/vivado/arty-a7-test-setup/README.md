# NEORV32 Test Setup for the Digilent Arty A7-35 FPGA Board

This setup provides a very simple script-based "demo setup" that allows to check out the NEORV32 processor on the Digilent Arty A7-35 board.
It uses the simplified [`neorv32_test_setup_bootloader.vhd`](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) top entity, which is a wrapper for the actual processor
top entity that provides a minimalistic interface (clock, reset, UART and 4 LEDs).

* FPGA Board: :books: [Digilent Arty A7-35 FPGA Board](https://reference.digilentinc.com/reference/programmable-logic/arty-a7/reference-manual)
* FPGA: Xilinx Artix-7 `XC7A35TICSG324-1L`
* Toolchain: Xilinx Vivado (tested with Vivado 2019.2)


## NEORV32 Configuration

:information_source: See the top entity [`rtl/test_setups/neorv32_test_setup_bootloader.vhd` ](https://github.com/stnolting/neorv32/blob/master/rtl/test_setups/neorv32_test_setup_bootloader.vhd) for
configuration and entity details and [`arty_a7_35_test_setup.xdc`](https://github.com/stnolting/neorv32/blob/master/boards/arty-a7-35-test-setup/arty_a7_35_test_setup.xdc)
for the according FPGA pin mapping.

* CPU: `rv32imcu_Zicsr` + 4 `HPM` (hardware performance monitors)
* Memory: 16kB instruction memory (internal IMEM), 8kB data memory (internal DMEM), bootloader ROM
* Peripherals: `GPIO`, `MTIME`, `UART0`, `WDT`
* Tested with version [`1.5.3.3`](https://github.com/stnolting/neorv32/blob/master/CHANGELOG.md)
* Clock: 100MHz from on-board oscillator
* Reset: Via dedicated on-board "RESET" button
* GPIO output port `gpio_o`
  * bits 0..3 are connected to the green on-board LEDs (LD4 - LD7); LD4 is the bootloader status LED
  * bits 4..7 are (not actually used) connected to PMOD `JA` connector pins 1-4
* UART0 signals `uart0_txd_o` and `uart0_rxd_i` are connected to the on-board USB-UART chip


