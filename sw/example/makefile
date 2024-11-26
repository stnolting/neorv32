.SUFFIXES:
.DEFAULT_GOAL := help

TOPTARGETS := exe clean_all check info all

SUBDIRS := $(wildcard */.)
# ignore dummy folders (starting with '~')
EXCLDIR := $(wildcard ~*/.)
# ignore some of the default projects/examples
EXCLDIR += eclipse/. dhrystone/. performance_tests/. float_corner_test/.
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
	@echo " exe       - create executables from all projects"
	@echo " all       - create executables and boot images from all projects"
	@echo " clean_all - clean up everything"
