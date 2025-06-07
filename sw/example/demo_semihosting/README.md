## NEORV32 Semihosting Example

Make sure to enable semihosting on the host side:

```
(gdb) monitor arm semihosting enable
(gdb) monitor arm semihosting_fileio enable
(gdb) monitor arm semihosting_basedir path/to/neorv32/sw/example/demo_semihosting
```

The last two command are required for accessing files on the host system.
In this example the `test.data` file from this folder is read by NEORV32.
