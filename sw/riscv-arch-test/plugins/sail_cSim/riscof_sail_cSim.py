import os
import re
import shutil
import subprocess
import shlex
import logging
import random
import string
import json
from string import Template

import riscof.utils as utils
from riscof.pluginTemplate import pluginTemplate
import riscof.constants as constants
from riscv_isac.isac import isac

logger = logging.getLogger()

class sail_cSim(pluginTemplate):
    __model__ = "sail_c_simulator"
    __version__ = "0.5.0"

    def __init__(self, *args, **kwargs):
        sclass = super().__init__(*args, **kwargs)

        config = kwargs.get('config')
        if config is None:
            logger.error("Config node for sail_cSim missing.")
            raise SystemExit(1)
        self.num_jobs = str(config['jobs'] if 'jobs' in config else 1)
        self.pluginpath = os.path.abspath(config['pluginpath'])
        self.sail_exe = os.path.join(config['PATH'] if 'PATH' in config else "","sail_riscv_sim")
        self.isa_spec = os.path.abspath(config['ispec']) if 'ispec' in config else ''
        self.platform_spec = os.path.abspath(config['pspec']) if 'ispec' in config else ''
        self.make = config['make'] if 'make' in config else 'make'
        logger.debug("SAIL CSim plugin initialised using the following configuration.")
        for entry in config:
            logger.debug(entry+' : '+config[entry])
        return sclass

    def initialise(self, suite, work_dir, archtest_env):
        self.suite = suite
        self.work_dir = work_dir
        self.objdump_cmd = 'riscv-none-elf-objdump -D {0} > {2};'
        self.compile_cmd = 'riscv-none-elf-gcc -march={0} \
         -static -mcmodel=medany -fvisibility=hidden -nostdlib -nostartfiles\
         -T '+self.pluginpath+'/env/link.ld\
         -I '+self.pluginpath+'/env/\
         -I ' + archtest_env

    def build(self, isa_yaml, platform_yaml):
        ispec = utils.load_yaml(isa_yaml)['hart0']
        self.xlen = ('64' if 64 in ispec['supported_xlen'] else '32')
        self.flen = ('64' if "D" in ispec["ISA"] else '32')
        self.isa_yaml_path = isa_yaml
        self.isa = 'rv' + self.xlen
        self.compile_cmd = self.compile_cmd+' -mabi='+('lp64 ' if 64 in ispec['supported_xlen'] else 'ilp32 ')
        if "I" in ispec["ISA"]:
            self.isa += 'i'
        if "M" in ispec["ISA"]:
            self.isa += 'm'
        if "A" in ispec["ISA"]:
            self.isa += 'a'
        if "F" in ispec["ISA"]:
            self.isa += 'f'
        if "D" in ispec["ISA"]:
            self.isa += 'd'
        if "C" in ispec["ISA"]:
            self.isa += 'c'
        objdump = "riscv-none-elf-objdump".format(self.xlen)
        if shutil.which(objdump) is None:
            logger.error(objdump+": executable not found. Please check environment setup.")
            raise SystemExit(1)
        compiler = "riscv-none-elf-gcc".format(self.xlen)
        if shutil.which(compiler) is None:
            logger.error(compiler+": executable not found. Please check environment setup.")
            raise SystemExit(1)
        if shutil.which(self.sail_exe) is None:
            logger.error(self.sail_exe + ": executable not found. Please check environment setup.")
            raise SystemExit(1)
        if shutil.which(self.make) is None:
            logger.error(self.make+": executable not found. Please check environment setup.")
            raise SystemExit(1)

        # ---- NEORV32-specific ----
        # Override default exception relocation list (traps for MTVAL being set zero)
        print("<plugin-sail_cSim> yaml-overwrite: overriding default SET_REL_TVAL_MSK macro")
        neorv32_override  = ' \"-DSET_REL_TVAL_MSK=(('
        neorv32_override += '(1<<CAUSE_MISALIGNED_LOAD)  | '
        neorv32_override += '(1<<CAUSE_LOAD_ACCESS)      | '
        neorv32_override += '(1<<CAUSE_MISALIGNED_STORE) | '
        neorv32_override += '(1<<CAUSE_STORE_ACCESS)       '
        neorv32_override += ') & 0xFFFFFFFF)\" '
        self.compile_cmd += neorv32_override

    def runTests(self, testList, cgf_file=None, header_file= None):
        if os.path.exists(self.work_dir+ "/Makefile." + self.name[:-1]):
            os.remove(self.work_dir+ "/Makefile." + self.name[:-1])
        make = utils.makeUtil(makefilePath=os.path.join(self.work_dir, "Makefile." + self.name[:-1]))
        make.makeCommand = self.make + ' -j' + self.num_jobs

        isa_yaml = utils.load_yaml(self.isa_yaml_path)

        # ---- NEORV32-specific ----
        ## [NOTE] Not supported by the used riscv_config, so configure this manually here
        print("<plugin-sail_cSim> yaml-overwrite: set static PMP configuration")
        pmp_flags = {}
        pmp_flags["pmp-grain"] = 0
        pmp_flags["pmp-count"] = 16

        try:
            sail_config = subprocess.run(["sail_riscv_sim", "--print-default-config"], check= True, text=True, capture_output=True)
            sail_config = json.loads(sail_config.stdout)
        except subprocess.CalledProcessError as e:
            print("sail_riscv_sim --print-default-config failed:", e.stderr)
            exit(1)
        except json.JSONDecodeError:
            print("sail_riscv_sim --print-default-config output is not valid JSON.")
            exit(1)

        sail_config["base"]["xlen"] = int(self.xlen)
        sail_config["memory"]["pmp"]["grain"] = pmp_flags["pmp-grain"]
        sail_config["memory"]["pmp"]["count"] = pmp_flags["pmp-count"]

        # ---- NEORV32-specific ----
        ## [NOTE] Not supported by the used riscv_config, so configure this manually here
        print("<plugin-sail_cSim> yaml-overwrite: disable misaligned access")
        sail_config["memory"]["misaligned"]["supported"] = False

        # Enabling extensions that are disabled by default
        sail_config["extensions"]["Sv32"]["supported"] = True
        sail_config["extensions"]["Zcf"]["supported"] = True

        # For User-configuration: Replace this variable with your configuration. "/home/riscv-arch-test/custom_sail_config.json"
        sail_config_path = os.path.join(self.pluginpath, 'env', 'sail_config.json')

        # Write the updated configuration back to the file
        with open(sail_config_path, 'w', encoding='utf-8') as file:
            json.dump(sail_config, file, indent=4)

        for file in testList:
            testentry = testList[file]
            test = testentry['test_path']
            test_dir = testentry['work_dir']
            test_name = test.rsplit('/',1)[1][:-2]

            elf = 'ref.elf'

            execute = "@cd "+testentry['work_dir']+";"

            cmd = self.compile_cmd.format(testentry['isa'].lower(), self.xlen) + ' ' + test + ' -o ' + elf
            compile_cmd = cmd + ' -D' + " -D".join(testentry['macros'])
            execute+=compile_cmd+";"

            execute += self.objdump_cmd.format(elf, self.xlen, 'ref.disass')
            sig_file = os.path.join(test_dir, self.name[:-1] + ".signature")

            execute += self.sail_exe + ' --config={0} --trace-all --signature-granularity=4  --test-signature={1} {2} > {3}.log 2>&1;'.format(sail_config_path, sig_file, elf, test_name)

            cov_str = ' '
            for label in testentry['coverage_labels']:
                cov_str+=' -l '+label

            cgf_mac = ' '
            header_file_flag = ' '
            if header_file is not None:
                header_file_flag = f' -h {header_file} '
                cgf_mac += ' -cm common '
                for macro in testentry['mac']:
                    cgf_mac+=' -cm '+macro

            if cgf_file is not None:
                coverage_cmd = 'riscv_isac --verbose info coverage -d \
                        -t {0}.log --parser-name c_sail -o coverage.rpt  \
                        --sig-label begin_signature  end_signature \
                        -e ref.elf -c {1} -x{2} -f{3} {4} {5} {6};'.format(\
                        test_name, ' -c '.join(cgf_file), self.xlen, self.flen, cov_str, header_file_flag, cgf_mac)
            else:
                coverage_cmd = ''


            execute+=coverage_cmd

            make.add_target(execute)
        make.execute_all(self.work_dir)
