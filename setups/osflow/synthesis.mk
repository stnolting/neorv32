${DEVICE_LIB}-obj08.cf: ${DEVICE_SRC}
	ghdl -a $(GHDL_FLAGS) --work=${DEVICE_LIB} ${DEVICE_SRC}

neorv32-obj08.cf: ${DEVICE_LIB}-obj08.cf ${NEORV32_SRC}
	ghdl -a $(GHDL_FLAGS) --work=neorv32 ${NEORV32_SRC}

work-obj08.cf: neorv32-obj08.cf ${DESIGN_SRC} ${BOARD_SRC}
	ghdl -a $(GHDL_FLAGS) --work=work ${DESIGN_SRC} ${BOARD_SRC}

${IMPL}.json: work-obj08.cf $(NEORV32_VERILOG_ALL)
	$(YOSYS) $(YOSYSFLAGS) \
	  -p \
	  "$(GHDLSYNTH) $(GHDL_FLAGS) --no-formal $(TOP); \
	  synth_${YOSYSSYNTH} \
	  -top $(TOP) $(YOSYSPIPE) \
	  -json $@" $(NEORV32_VERILOG_ALL) 2>&1 | tee yosys-report.txt
