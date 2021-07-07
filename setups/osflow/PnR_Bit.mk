${IMPL}.${PNR2BIT_EXT}: $(IMPL).json $(CONSTRAINTS)
	$(NEXTPNR) \
	  $(PNRFLAGS) \
	  --$(CONSTRAINTS_FORMAT) $(CONSTRAINTS) \
	  --json $(IMPL).json \
	  --${NEXTPNR_OUT} $@ 2>&1 | tee nextpnr-report.txt

${IMPL}.bit: ${IMPL}.${PNR2BIT_EXT}
	$(PACKTOOL) $< $@

ifeq ($(DEVICE_SERIES),ecp5)
${IMPL}.svf: ${IMPL}.${PNR2BIT_EXT}
	$(PACKTOOL) $(PACKARGS) --svf $@ $<
endif
