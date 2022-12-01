# NEORV32 Bit-Manipulation `B` Extension

The provided test program `main.c` verifies all currently implemented instruction by checking the
results against a pure-software emulation model. The emulation functions as well as the available
**intrinsics** for the sub-extension are located in `neorv32_b_extension_intrinsics.h`.

:information_source: See the according section of the NEORV32 data sheet for more information.


## Exemplary Test Output

```
<<< NEORV32 Bit-Manipulation Extension ('B') Test >>>

Starting bit-manipulation extension tests (1000000 test cases per instruction)...

--------------------------------------------
Zbb - Basic bit-manipulation instructions
--------------------------------------------

ANDN:
Errors: 0/1000000 [ok]

ORN:
Errors: 0/1000000 [ok]

XNOR:
Errors: 0/1000000 [ok]

CLZ:
Errors: 0/1000000 [ok]

CTZ:
Errors: 0/1000000 [ok]

CPOP:
Errors: 0/1000000 [ok]

MAX:
Errors: 0/1000000 [ok]

MAXU:
Errors: 0/1000000 [ok]

MIN:
Errors: 0/1000000 [ok]

MINU:
Errors: 0/1000000 [ok]

SEXT.B:
Errors: 0/1000000 [ok]

SEXT.H:
Errors: 0/1000000 [ok]

ZEXT.H:
Errors: 0/1000000 [ok]

ROL:
Errors: 0/1000000 [ok]

ROR:
Errors: 0/1000000 [ok]

RORI (imm=20):
Errors: 0/1000000 [ok]

ORCB:
Errors: 0/1000000 [ok]

REV8:
Errors: 0/1000000 [ok]


--------------------------------------------
Zba - Address-generation instructions
--------------------------------------------

SH1ADD:
Errors: 0/1000000 [ok]

SH2ADD:
Errors: 0/1000000 [ok]

SH3ADD:
Errors: 0/1000000 [ok]


--------------------------------------------
Zbs - Single-bit instructions
--------------------------------------------

BCLR:
Errors: 0/1000000 [ok]

BCLRI (imm=20):
Errors: 0/1000000 [ok]

BEXT:
Errors: 0/1000000 [ok]

BEXTI (imm=20):
Errors: 0/1000000 [ok]

BINV:
Errors: 0/1000000 [ok]

BINVI (imm=20):
Errors: 0/1000000 [ok]

BSET:
Errors: 0/1000000 [ok]

BSETI (imm=20):
Errors: 0/1000000 [ok]


--------------------------------------------
Zbc - Carry-less multiplication instructions
--------------------------------------------

NOTE: The emulation functions will take quite some time to execute.

CLMUL:
Errors: 0/1000000 [ok]

CLMULH:
Errors: 0/1000000 [ok]

CLMULR:
Errors: 0/1000000 [ok]


B extension tests completed.
```
