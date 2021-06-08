${IMPL}.asc: $(IMPL).json $(PCF)
	$(NEXTPNR) \
	  $(PNRFLAGS) \
	  --pcf $(PCF) \
	  --json $(IMPL).json \
	  --asc $@ 2>&1 | tee nextpnr-report.txt

${IMPL}.bit: $(IMPL).asc
	$(ICEPACK) $< $@
