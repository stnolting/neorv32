# NEORV32 `Zfinx` Floating-Point Extension

The NEORV32 floating-point unit (FPU) implements the `Zfinx` RISC-V extension. The extensions can be enabled via the `CPU_EXTENSION_RISCV_Zfinx` top configuration generic.

The RISC-V `Zfinx` single-precision floating-point extensions uses the integer register file `x` instead of the dedicated floating-point `f` register file (which is
defined by the RISC-V `F` single-precision floating-point extension). Hence, the standard data transfer instructions from the `F` extension are **not** available in `Zfinx`:

* floating-point load/store operations (`FLW`, `FSW`) and their compressed versions
* integer register file `x` <-> floating point register file `f` move operations (`FMV.W.X`, `FMV.X.W`)

:information_source: More information regarding the RISC-V `Zfinx` single-precision floating-point extension can be found in the official GitHub repo:
[`github.com/riscv/riscv-zfinx`](https://github.com/riscv/riscv-zfinx).

:warning: The RISC-V `Zfinx` extension is not officially ratified yet, but it is assumed to remain unchanged. Hence, it is not supported by the upstream RISC-V GCC port.
Make sure you **do not** use the `f` ISA attribute when compiling applications that use floating-point arithmetic (`MARCH=rv32i*f*` is **NOT ALLOWED!**).


### :warning: FPU Limitations

* The FPU **does not support subnormal numbers** yet. Subnormal FPU inputs and subnormal FPU results are always *flushed to zero*. The *classify* instruction `FCLASS` will never set the "subnormal" mask bits.
* Rounding mode `ob100` "round to nearest, ties to max magnitude" is not supported yet (this and all invalid rounding mode configurations behave as "round towards zero" (truncation)).


## Intrinsic Library

The NEORV32 `Zfinx` floating-point extension can still be used using the provided **intrinsic library**. This library uses "custom" inline assmbly instructions
wrapped within normal C-language functions. Each original instruction of the extension can be utilized using an according intrinsic function.

For example, the floating-point addition instruction `FADD.S` can be invoked using the according intrinsic function:

```c
float riscv_intrinsic_fadds(float rs1, float rs2)
```

The pure-software emulation instruction, which uses the standard built-in functions to execute all floating-point operations, is available via wrapper function. The
emulation function for the `FADD.S` instruction is:

```c
float riscv_emulate_fadds(float rs1, float rs2)
```

The emulation functions as well as the available intrinsics for the `Zfinx` extension are located in `neorv32_zfinx_extension_intrinsics.h`.

The provided test program `main.c` verifies all currently implemented `Zfinx` instructions by checking the functionality against the pure software-based emulation model
(GCC soft-float library).


## Resources

* Great page with online calculators for floating-point arithmetic: [http://www.ecs.umass.edu/ece/koren/arith/simulator/](http://www.ecs.umass.edu/ece/koren/arith/simulator/)
* A handy tool for visualizing floating-point numbers in their binary representation: [https://www.h-schmidt.net/FloatConverter/IEEE754.html](https://www.h-schmidt.net/FloatConverter/IEEE754.html)
* This helped me to understand what results the different FPU operation generate when having "special" inputs like NaN: [https://techdocs.altium.com/display/FPGA/IEEE+754+Standard+-+Overview](https://techdocs.altium.com/display/FPGA/IEEE+754+Standard+-+Overview)
