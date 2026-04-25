# NEORV32 `Zfinx` Floating-Point Extension

The NEORV32 floating-point unit (FPU) implements the `Zfinx` RISC-V extension. The extension can be
enabled via the `CPU_EXTENSION_RISCV_Zfinx` top configuration generic.

The RISC-V `Zfinx` single-precision floating-point extensions uses the integer register file `x` instead
of a dedicated floating-point `f` register file (which is defined by the RISC-V `F` single-precision
floating-point extension). Hence, the standard data transfer instructions from the `F` extension are
**not** available in `Zfinx`:

* floating-point load/store operations (`FLW`, `FSW`) and their compressed versions
* integer register file `x` <-> floating point register file `f` move operations (`FMV.W.X`, `FMV.X.W`)

:information_source: See the according section of the NEORV32 data sheet for more information.


## Intrinsic Library

The NEORV32 `Zfinx` floating-point extension can still be used using the provided **intrinsic library**. This
library uses "custom" inline assembly instructions wrapped within normal C-language functions. Each original
instruction of the extension can be utilized using an according intrinsic function.

For example, the floating-point addition instruction `FADD.S` can be invoked using the according intrinsic function:

```c
float riscv_intrinsic_fadds(float rs1, float rs2)
```

The pure-software emulation instruction, which uses the standard built-in functions to execute all
floating-point operations, is available via wrapper function. The emulation function for the `FADD.S` instruction is:

```c
float riscv_emulate_fadds(float rs1, float rs2)
```

The emulation functions as well as the available intrinsics for the `Zfinx` extension are located in
`neorv32_zfinx_extension_intrinsics.h`. The provided test program `main.c` verifies all currently implemented
`Zfinx` instructions by checking the functionality against the pure software-based emulation model (GCC soft-float library).


## Exemplary Test Output

```
<<< Zfinx extension test >>>
SILENT_MODE enabled (only showing actual errors)
Test cases per instruction: 1000000
NOTE: The NEORV32 FPU does not support subnormal numbers yet. Subnormal numbers are flushed to zero.


#0: FCVT.S.WU (unsigned integer to float)...
Errors: 0/1000000 [ok]

#1: FCVT.S.W (signed integer to float)...
Errors: 0/1000000 [ok]

#2: FCVT.WU.S (float to unsigned integer)...
Errors: 0/1000000 [ok]

#3: FCVT.W.S (float to signed integer)...
Errors: 0/1000000 [ok]

#4: FADD.S (addition)...
Errors: 0/1000000 [ok]

#5: FSUB.S (subtraction)...
Errors: 0/1000000 [ok]

#6: FMUL.S (multiplication)...
Errors: 0/1000000 [ok]

#7: FMIN.S (select minimum)...
Errors: 0/1000000 [ok]

#8: FMAX.S (select maximum)...
Errors: 0/1000000 [ok]

#9: FEQ.S (compare if equal)...
Errors: 0/1000000 [ok]

#10: FLT.S (compare if less-than)...
Errors: 0/1000000 [ok]

#11: FLE.S (compare if less-than-or-equal)...
Errors: 0/1000000 [ok]

#12: FSGNJ.S (sign-injection)...
Errors: 0/1000000 [ok]

#13: FSGNJN.S (sign-injection NOT)...
Errors: 0/1000000 [ok]

#14: FSGNJX.S (sign-injection XOR)...
Errors: 0/1000000 [ok]

#15: FCLASS.S (classify)...
Errors: 0/1000000 [ok]

# unsupported FDIV.S (division) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x000006A8, INST=0x18A484D3 </RTE>
[ok]

# unsupported FSQRT.S (square root) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x000006E0, INST=0x580484D3 </RTE>
[ok]

# unsupported FMADD.S (fused multiply-add) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x0000071E, INST=0x1EA484C3 </RTE>
[ok]

# unsupported FMSUB.S (fused multiply-sub) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x0000075C, INST=0x1EA484C7 </RTE>
[ok]

# unsupported FNMSUB.S (fused negated multiply-sub) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x0000079A, INST=0x1EA484CF </RTE>
[ok]

# unsupported FNMADD.S (fused negated multiply-add) [illegal instruction]...
<RTE> Illegal instruction @ PC=0x000007D8, INST=0x1EA484CF </RTE>
[ok]

[Zfinx extension verification successful!]
```
