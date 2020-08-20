#################################################################################################
# << NEORV32 - All-Projects Makefile >>                                                         #
# ********************************************************************************************* #
# Make sure to add the riscv GCC compiler's bin folder to your PATH variable.                   #
# ********************************************************************************************* #
# BSD 3-Clause License                                                                          #
#                                                                                               #
# Copyright (c) 2020, Stephan Nolting. All rights reserved.                                     #
#                                                                                               #
# Redistribution and use in source and binary forms, with or without modification, are          #
# permitted provided that the following conditions are met:                                     #
#                                                                                               #
# 1. Redistributions of source code must retain the above copyright notice, this list of        #
#    conditions and the following disclaimer.                                                   #
#                                                                                               #
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
#    conditions and the following disclaimer in the documentation and/or other materials        #
#    provided with the distribution.                                                            #
#                                                                                               #
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
#    endorse or promote products derived from this software without specific prior written      #
#    permission.                                                                                #
#                                                                                               #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
# OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
# GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
# OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
# ********************************************************************************************* #
# The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
#################################################################################################


#-------------------------------------------------------------------------------
# Make defaults and targets
#-------------------------------------------------------------------------------
.SUFFIXES:
.DEFAULT_GOAL := help

TOPTARGETS := exe clean_all check info all

SUBDIRS := $(wildcard */.)
# ignore dummy folders (starting with '~')
DUMMYDIRS := $(wildcard ~*/.)
SUBDIRS := $(filter-out $(DUMMYDIRS), $(SUBDIRS))

$(TOPTARGETS): $(SUBDIRS)
$(SUBDIRS):
	@set -e
	@$(MAKE) -C $@ $(MAKECMDGOALS)

.PHONY: $(TOPTARGETS) $(SUBDIRS)


#-------------------------------------------------------------------------------
# Help
#-------------------------------------------------------------------------------
help:
	@echo "Build / clean up all projects"
	@echo "Targets:"
	@echo " help      - show this text"
	@echo " check     - check toolchain"
	@echo " info      - show makefile configuration"
	@echo " exe       - create executables from all projects"
	@echo " all       - create executables and boot images from all projects"
	@echo " clean_all - clean up everything"

