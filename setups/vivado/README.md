# NEORV32 Xilinx Vivado Example Setups

## How To Run

The `create_project.tcl` TCL script in the board subdirectories can be used for creating a complete Vivado project and for running the implementation.
If not already available, this script will create a `work` folder in those subdirectories.

Note that you may need to install support for your particular development board through "XHub Stores" menu item within Vivado prior to sourcing the `create_project.tcl` script.

### Batch mode

Execute `vivado -mode batch -nojournal -nolog -source create_project.tcl` from the board subdir.
The project will be created and implementation will be run until generation of `work/neorv32_test_setup.runs/impl_1/neorv32_test_setup.bit`.

### GUI

1. start Vivado (in GUI mode)
2. click on "TCL Console" at the bottom
3. use the console to naviagte to the boards folder. For example: `cd .../neorv32/setups/vivado/arty-a7-test-setup`
4. execute `source create_project.tcl` - this will create the actual Vivado project in `work`
5. when the Vivado project has openend, Implementation will run and a bitstream will be generated.
6. maybe a prompt will notify about it.

### Programming the Bitstream

1. open the "Hardware Manager" (maybe a prompt will ask for that)
2. click on "Open target/Auto Connect"
3. click on "Program device" and select `work/neorv32_test_setup.runs/impl_1/neorv32_test_setup.bit`; click "Program"
4. use a serial terminal (like :earth_asia: [Tera Term](https://ttssh2.osdn.jp/index.html.en)) to connect to the USB-UART interface using the following configuration:
19200 Baud, 8 data bits, 1 stop bit, no parity bits, no transmission / flow control protocol (raw bytes only), newline on `\r\n` (carriage return & newline)
5. now you can communicate with the bootloader console and upload a new program. Check out the [example programs](https://github.com/stnolting/neorv32/tree/master/sw/example)
and see section "Let's Get It Started" of the :page_facing_up: [NEORV32 data sheet](https://raw.githubusercontent.com/stnolting/neorv32/master/docs/NEORV32.pdf) for further resources.
