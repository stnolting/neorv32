# NEORV32 `Zfinx` Floating-Point normalizer corner test

:bulb: See https://github.com/stnolting/neorv32/pull/528 by [@mikaelsky](https://github.com/mikaelsky).

This test case highlights instruction time outs with the FPU normalizer circuits. Instructions impacted are:
FCVT.WU.S
FCVT.W.S
FADD.S
FSUB.S

The example code runs through all possible exponent values and test the indivual instructions to determine whether the normalizer triggers an instruction time out of >=128 clock cycles.

For exponent sizes that fail an error output will be generated.

## Exemplary Test Output

```
Booting from 0x00000000...

<<< Zfinx extension corner test >>>
SILENT_MODE enabled (only showing actual errors)
NOTE: The NEORV32 FPU does not support subnormal numbers yet. Subnormal numbers are flushed to zero.


#0: FCVT.WU.S (float to unsigned integer)...
<RTE> Illegal instruction @ PC=0x0000027C, INST=0xC01487D3 </RTE>
[FAILED]
Conversion of opa with 254 exponent timed out
<RTE> Illegal instruction @ PC=0x0000027C, INST=0xC01487D3 </RTE>
[FAILED]
Conversion of opa with 253 exponent timed out
<RTE> Illegal instruction @ PC=0x0000027C, INST=0xC01487D3 </RTE>
[FAILED]
Conversion of opa with 252 exponent timed out
<RTE> Illegal instruction @ PC=0x0000027C, INST=0xC01487D3 </RTE>
[FAILED]
Conversion of opa with 251 exponent timed out
<RTE> Illegal instruction @ PC=0x0000027C, INST=0xC01487D3 </RTE>
[FAILED]
Conversion of opa with 250 exponent timed out

#1: FCVT.W.S (float to signed integer)...
<RTE> Illegal instruction @ PC=0x000002E4, INST=0xC00487D3 </RTE>
[FAILED]
Conversion of opa with 254 exponent timed out
<RTE> Illegal instruction @ PC=0x000002E4, INST=0xC00487D3 </RTE>
[FAILED]
Conversion of opa with 253 exponent timed out
<RTE> Illegal instruction @ PC=0x000002E4, INST=0xC00487D3 </RTE>
[FAILED]
Conversion of opa with 252 exponent timed out
<RTE> Illegal instruction @ PC=0x000002E4, INST=0xC00487D3 </RTE>
[FAILED]
Conversion of opa with 251 exponent timed out
<RTE> Illegal instruction @ PC=0x000002E4, INST=0xC00487D3 </RTE>
[FAILED]
Conversion of opa with 250 exponent timed out

#2: FADD.S (addition)...
<RTE> Illegal instruction @ PC=0x00000348, INST=0x009907D3 </RTE>
[FAILED]
Addition using opa with 254 exponent and opb with 1 exponent timed out
<RTE> Illegal instruction @ PC=0x00000348, INST=0x009907D3 </RTE>
[FAILED]
Addition using opa with 254 exponent and opb with 2 exponent timed out
<RTE> Illegal instruction @ PC=0x00000348, INST=0x009907D3 </RTE>
[FAILED]
Addition using opa with 254 exponent and opb with 3 exponent timed out
....
Addition using opa with 254 exponent and opb with 138 exponent timed out
<RTE> Illegal instruction @ PC=0x00000348, INST=0x009907D3 </RTE>
[FAILED]
Addition using opa with 254 exponent and opb with 139 exponent timed out
<RTE> Illegal instruction @ PC=0x00000348, INST=0x009907D3 </RTE>
[FAILED]
Addition using opa with 254 exponent and opb with 140 exponent timed out

#3: FSUB.S (subtraction)...
<RTE> Illegal instruction @ PC=0x000003B0, INST=0x089907D3 </RTE>
[FAILED]
Subtraction using opa with 254 exponent and opb with 1 exponent timed out
<RTE> Illegal instruction @ PC=0x000003B0, INST=0x089907D3 </RTE>
[FAILED]
Subtraction using opa with 254 exponent and opb with 2 exponent timed out
....
Subtraction using opa with 254 exponent and opb with 140 exponent timed out
<RTE> Illegal instruction @ PC=0x000003B0, INST=0x089907D3 </RTE>
[FAILED]
Subtraction using opa with 254 exponent and opb with 141 exponent timed out
<RTE> Illegal instruction @ PC=0x000003B0, INST=0x089907D3 </RTE>
[FAILED]
Subtraction using opa with 254 exponent and opb with 142 exponent timed out

[ZFINX EXTENSION VERIFICATION FAILED!]
292 errors in 4 test cases