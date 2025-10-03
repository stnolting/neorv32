## NEORV32 OpenOCD Scripts

* `neorv32.cfg` For the default **single-core** processor configuration
* `neorv32.dual_core.cfg` For the **SMP dual-core** processor configuration

Example:

```bash
neorv32/sw/openocd$ openocf -f neorv32.cfg
```

Helper scripts in [`lib`](lib) are called by the main configuration files
in a specific order to setup the target:

1. `lib/interface.cfg` Physical (JTAG) adapter configuration.
2. `lib/target.cfg` CPU core(s) and GDB configuration.
3. `lib/authenticate.cfg` Authenticate debug access via the RISC-V DM authentication commands
using the _default_ NEORV32 authenticator.
4. `lib/start.cfg` Reset and halt target.

To adapt it to your own design, you can customize the included files or replace them entirely.
