## NEORV32 OpenOCD Scripts

* `openocd_neorv32.cfg` For the default **single-core** processor configuration
* `openocd_neorv32.dual_core.cfg` For the **SMP dual-core** processor configuration

### Helper Scripts

The _helper scripts_ are called by the main OpenOCD configuration files.
Hence, these scripts are not meant for stand-alone operation.

* `authentication.cfg` Authenticate debug access via the RISC-V DM authentication commands.
Modify this file (but not the helper procedures) if you are using a
**custom/non-default authenticator** processor hardware module.
* `interface.cfg` Physical adapter configuration. Adjust this file to match your specific adapter board.
* `target.cfg` CPU core(s) and GDB configuration. This file should not be altered.
