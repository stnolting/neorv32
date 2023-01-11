RTL_DIR := ../../rtl/core

DUT := neorv32_gpio

.PHONY: cover bmc prove all clean

all: cover bmc prove

cover bmc prove: ${RTL_DIR}/${DUT}.vhd symbiyosys.sby
	sby --yosys "yosys -m ghdl" -f -d work/${DUT}-$@ symbiyosys.sby $@

clean:
	rm -rf work
