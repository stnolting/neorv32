.SUFFIXES:
.DEFAULT_GOAL := help

TOPTARGETS := elf exe clean clean_all check info all

SUBDIRS := $(wildcard */.)
EXCLDIR := $(wildcard ~*/.)
SUBDIRS := $(filter-out $(EXCLDIR), $(SUBDIRS))

$(TOPTARGETS): $(SUBDIRS)
$(SUBDIRS):
	@set -e
	@$(MAKE) -C $@ $(MAKECMDGOALS)

.PHONY: $(TOPTARGETS) $(SUBDIRS)

help:
	@echo "Build / clean up all projects"
	@echo "Targets:"
	@echo " help      - show this text"
	@echo " check     - check toolchain"
	@echo " info      - show makefile configuration"
	@echo " all       - create everything from all projects"
	@echo " clean_all - clean up everything"
