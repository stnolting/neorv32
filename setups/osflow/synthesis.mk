ice40up-obj08.cf: ${ICE40_SRC}
	ghdl -a $(GHDL_FLAGS) --work=iCE40UP ${ICE40_SRC}

neorv32-obj08.cf: ice40up-obj08.cf ${NEORV32_SRC}
	ghdl -a $(GHDL_FLAGS) --work=neorv32 ${NEORV32_SRC}

work-obj08.cf: neorv32-obj08.cf ${DESIGN_SRC} ${BOARD_SRC}
	ghdl -a $(GHDL_FLAGS) --work=work ${DESIGN_SRC} ${BOARD_SRC}

${IMPL}.json: work-obj08.cf
	$(YOSYS) $(YOSYSFLAGS) \
	  -p \
	  "$(GHDLSYNTH) $(GHDL_FLAGS) --no-formal $(TOP); \
	  synth_ice40 \
	  -top $(TOP) \
	  -dsp \
	  -json $@" 2>&1 | tee yosys-report.txt
