## NEORV32 Software Library

This folder contains the NEORV32 hardware abstraction layer (HAL), runtime
environment, and several helper/auxiliary functions. Application programs
should only include the *main NEORV32 define file* `neorv32.h`. This file
automatically includes all other header files.

```c
#include <neorv32.h>
```
