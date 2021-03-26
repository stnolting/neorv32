# NEORV32 `Zfinx` Floating-Point Extension

The RISC-V `Zfinx` single-precision floating-point extensions uses the integer register file `x` instead of the dedicated floating-point `f` register file (which is
defined by the RISC-V `F` single-precision floating-point extension). Hence, the standard data transfer instructions from the `F` extension are **not** available in `Zfinx`:

* floating-point load/store operations (`FLW`, `FSW`) and their compressed versions
* integer register file `x` <-> floating point register file `f` move operations (`FMV.W.X`, `FMV.X.W`)


:information_source: More information regarding the RISC-V `Zfinx` single-precision floating-point extension can be found in the officail GitHub repo:
[`github.com/riscv/riscv-zfinx`](https://github.com/riscv/riscv-zfinx).

:warning: The RISC-V `Zfinx` extension is not officially ratified yet, but it is assumed to remain unchanged. Hence, it is not supported by the upstream RISC-V GCC port.
Make sure you **do not** use the `f` ISA attribute when compiling applications that use floating-point arithmetic (`-march=rv32i*f*` is **NOT ALLOWED!**).


## Intrinsic Library

The NEORV32 `Zfinx` floating-point extension can still be used using the provided **intrinsic library**. This library uses "custom" inline assmbly instructions
wrapped within normal C-language functions. Each original instruction of the extension can be utilized using an according intrinsic function.

For example, the floating-point addition instruction `FADD.S` can be invoked using the according intrinsic function:

```c
float riscv_intrinsic_fadds(float rs1, float rs2)
```

The pure-software emulation instruction, which uses the standard builtin functions to execute all floating-point operations, is available via wrapper function. The
emulation function for the `FADD.S` instruction is:

```c
float riscv_emulate_fadds(float rs1, float rs2)
```

The emulation functions as well as the available intrinsics for the `Zfinx` extension are located in `neorv32_zfinx_extension_intrinsics.h`.

The provided test program `main.c` verifies all currently implemented `Zfinx` instructions by checking the functionality against the pure software-based emulation model
(GCC soft-float library).
