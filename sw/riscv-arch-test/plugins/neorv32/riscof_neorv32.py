import os
import re
import shutil
import subprocess
import shlex
import logging
import random
import string
from string import Template
import sys

import riscof.utils as utils
import riscof.constants as constants
from riscof.pluginTemplate import pluginTemplate

logger = logging.getLogger()

# Configuration
RVGCCPREFIX = "riscv-none-elf-"
GHDLEXE = "ghdl"
RTLCORE = "../../rtl/core"
TESTBENCH = "rvtest_tb"

class neorv32(pluginTemplate):
    __model__ = "neorv32"
    __version__ = "latest"

    def __init__(self, *args, **kwargs):
        sclass = super().__init__(*args, **kwargs)

        config = kwargs.get('config')

        # If the config node for this DUT is missing or empty. Raise an error. At minimum we need
        # the paths to the ispec and pspec files
        if config is None:
            print("Please enter input file paths in configuration.")
            raise SystemExit(1)

        # prepare simulation (GHDL)
        execute = f"{GHDLEXE} -i --work=neorv32 --std=08 {RTLCORE}/*.vhd {TESTBENCH}.vhd"
        execute += " && "
        execute += f"{GHDLEXE} -m --std=08 --work=neorv32 {TESTBENCH}"
        logger.debug('DUT executing ' + execute)
        utils.shellCommand(execute).run()

        # Number of parallel jobs that can be spawned off by RISCOF
        # for various actions performed in later functions, specifically to run the tests in
        # parallel on the DUT executable. Can also be used in the build function if required.
        self.num_jobs = str(config['jobs'] if 'jobs' in config else 1)

        # Path to the directory where this python file is located. Collect it from the config.ini
        self.pluginpath=os.path.abspath(config['pluginpath'])

        # Collect the paths to the  riscv-config absed ISA and platform yaml files. One can choose
        # to hardcode these here itself instead of picking it from the config.ini file.
        self.isa_spec = os.path.abspath(config['ispec'])
        self.platform_spec = os.path.abspath(config['pspec'])

        #We capture if the user would like the run the tests on the target or
        #not. If you are interested in just compiling the tests and not running
        #them on the target, then following variable should be set to False
        if 'target_run' in config and config['target_run']=='0':
            self.target_run = False
        else:
            self.target_run = True

        # Return the parameters set above back to RISCOF for further processing.
        return sclass

    def initialise(self, suite, work_dir, archtest_env):

       # capture the working directory. Any artifacts that the DUT creates should be placed in this
       # directory. Other artifacts from the framework and the Reference plugin will also be placed
       # here itself.
       self.work_dir = work_dir

       # capture the architectural test-suite directory.
       self.suite_dir = suite

       # Note the march is not hardwired here, because it will change for each
       # test. Similarly the output elf name and compile macros will be assigned later in the
       # runTests function
       self.compile_cmd = f"{RVGCCPREFIX}gcc"
       self.compile_cmd += ' -march={0} \
         -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles -g\
         -T '+self.pluginpath+'/env/link.ld\
         -I '+self.pluginpath+'/env/\
         -I ' + archtest_env + ' {2} -o {3} {4}'

    def build(self, isa_yaml, platform_yaml):

      # load the isa yaml as a dictionary in python.
      ispec = utils.load_yaml(isa_yaml)['hart0']

      # capture the XLEN value by picking the max value in 'supported_xlen' field of isa yaml. This
      # will be useful in setting integer value in the compiler string (if not already hardcoded);
      self.xlen = ('64' if 64 in ispec['supported_xlen'] else '32')
      self.compile_cmd = self.compile_cmd+' -mabi='+('lp64 ' if 64 in ispec['supported_xlen'] else 'ilp32 ')

      # Override default exception relocation list (traps for MTVAL being set to zero)
      print("<plugin-neorv32> yaml-overwrite: overriding default SET_REL_TVAL_MSK macro")
      neorv32_override  = ' \"-DSET_REL_TVAL_MSK=(('
      neorv32_override += '(1<<CAUSE_MISALIGNED_LOAD)  | '
      neorv32_override += '(1<<CAUSE_LOAD_ACCESS)      | '
      neorv32_override += '(1<<CAUSE_MISALIGNED_STORE) | '
      neorv32_override += '(1<<CAUSE_STORE_ACCESS)       '
      neorv32_override += ') & 0xFFFFFFFF)\" '
      self.compile_cmd += neorv32_override

    def runTests(self, testList):

      # we will iterate over each entry in the testList. Each entry node will be referred to by the
      # variable testname.
      test_cnt = 0;
      for testname in testList:

          logger.debug('Running Test: {0} on DUT'.format(testname))
          # for each testname we get all its fields (as described by the testList format)
          testentry = testList[testname]

          # we capture the path to the assembly file of this test
          test = testentry['test_path']

          # capture the directory where the artifacts of this test will be dumped/created.
          test_dir = testentry['work_dir']

          # name of the elf file after compilation of the test
          elf = 'main.elf'

          # name of the signature file as per requirement of RISCOF. RISCOF expects the signature to
          # be named as DUT-<dut-name>.signature. The below variable creates an absolute path of
          # signature file.
          sig_file = os.path.join(test_dir, self.name[:-1] + ".signature")

          # for each test there are specific compile macros that need to be enabled. The macros in
          # the testList node only contain the macros/values. For the gcc toolchain we need to
          # prefix with "-D". The following does precisely that.
          compile_macros = ' -D' + " -D".join(testentry['macros'])

          # collect the march string required for the compiler
          marchstr = testentry['isa'].lower()

          # substitute all variables in the compile command that we created in the initialize
          # function
          cmd = self.compile_cmd.format(marchstr, self.xlen, test, elf, compile_macros)

          # just a simple logger statement that shows up on the terminal
          logger.debug('Compiling test: ' + test)

          # the following command spawns a process to run the compile command. Note here, we are
          # changing the directory for this command to that pointed by test_dir. If you would like
          # the artifacts to be dumped else where change the test_dir variable to the path of your
          # choice.
          utils.shellCommand(cmd).run(cwd=test_dir)

          # generate plain binary memory image
          execute = f"{RVGCCPREFIX}objcopy -I elf32-little {test_dir}/{elf} -j .text -O binary {test_dir}/main.bin"
          logger.debug('DUT executing ' + execute)
          utils.shellCommand(execute).run()

          # print current test
          test_cnt = test_cnt + 1
          print(f"[{test_cnt}/{len(testList)}] {os.path.basename(testname)}")

          # run GHDL simulation
          execute = f"{GHDLEXE} -r --std=08 --work=neorv32 {TESTBENCH} -gTEST_DIR={test_dir}/ "
          execute += f"--max-stack-alloc=0 --ieee-asserts=disable --assert-level=error --stop-time=4ms "
          logger.debug('DUT executing ' + execute)
          utils.shellCommand(execute).run()

      # if target runs are not required then we simply exit as this point after running all
      # the makefile targets.
      if not self.target_run:
          raise SystemExit
