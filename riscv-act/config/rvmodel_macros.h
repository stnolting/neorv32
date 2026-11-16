// NEORV32-specifc ACT macros

#ifndef _RVMODEL_MACROS_H
#define _RVMODEL_MACROS_H

#define STANDARD_SM_SUPPORTED

#define RVMODEL_DATA_SECTION                      \
  .pushsection .tohost,"aw",@progbits;            \
  .align 8; .global tohost; tohost: .dword 0;     \
  .align 8; .global fromhost; fromhost: .dword 0; \
  .popsection;

##### STARTUP #####

# Perform boot operations. Can be empty or left undefined unless needed for
# DUT-specific behavior such as turning on a memory controller or
# initializing custom state.
//#define RVMODEL_BOOT

# No custom boot-to-m-mode required.
//#define RVMODEL_BOOT_TO_MMODE

##### TERMINATION #####

# Terminate test with a pass indication.
# When the test is run in simulation, this should end the simulation.
#define RVMODEL_HALT_PASS \
  li x1, 0xF0000000;      \
  li x2, 0x12345678;      \
  sw x2, 0(x1);           \
  j .;                    \

# Terminate test with a fail indication.
# When the test is run in simulation, this should end the simulation.
#define RVMODEL_HALT_FAIL \
  li x1, 0xF0000000;      \
  li x2, 0;               \
  sw x2, 0(x1);           \
  j .;                    \

##### IO #####

# Example UART implementation.
# Expects a PC16550-compatible UART.
# Change these addresses to match your memory map
.EQU UART0_BASE_ADDR, 0xFFF50000
.EQU UART0_CTRL, (UART0_BASE_ADDR + 0)
.EQU UART0_DATA, (UART0_BASE_ADDR + 4)

# Initialization steps needed prior to writing to the console.
#define RVMODEL_IO_INIT(_R1, _R2, _R3)

# Prints a null-terminated string using a DUT specific mechanism.
#define RVMODEL_IO_WRITE_STR(_R1, _R2, _R3, _STR_PTR) \
1:                                                    \
  lbu  _R1, 0(_STR_PTR);                              \
  beq  _R1, x0, 2f;                                   \
  li   _R2, 0xF0000004;                               \
  sb   _R1, 0(_R2);                                   \
  addi _STR_PTR, _STR_PTR, 1;                         \
  j 1b;                                               \
2:

##### Access Fault #####

#define RVMODEL_ACCESS_FAULT_ADDRESS 0x00000000

##### MTVEC Alignment #####

#define RVMODEL_MTVEC_ALIGN 7

##### Interrupt Latency #####

#define RVMODEL_INTERRUPT_LATENCY 10

##### Machine Timer & Interrupts #####

#define RVMODEL_TIMER_INT_SOON_DELAY 20000
#define RVMODEL_MAX_CYCLES_PER_TIMER_TICK 1

#define MEXT_BASE_ADDRESS  0xF0000008
#define CLINT_BASE_ADDRESS 0xFFF40000

#define RVMODEL_MTIME_ADDRESS    (CLINT_BASE_ADDRESS + 0xBFF8)
#define RVMODEL_MTIMECMP_ADDRESS (CLINT_BASE_ADDRESS + 0x4000)
// [hack] SAIL seems to use its own RVMODEL_MTIME_ADDRESS
// -> make RVMODEL_MSIP_ADDRESS relative to RVMODEL_MTIME_ADDRESS
#define RVMODEL_MSIP_ADDRESS     (RVMODEL_MTIME_ADDRESS - 0xBFF8)

#define RVMODEL_SET_MEXT_INT(_R1, _R2) \
  li _R1, 1;                           \
  li _R2, MEXT_BASE_ADDRESS;           \
  sw _R1, 0(_R2);

#define RVMODEL_CLR_MEXT_INT(_R1, _R2) \
  li _R2, MEXT_BASE_ADDRESS;           \
  sw x0, 0(_R2);

##### Supervisor Interrupts (not available) #####

#define RVMODEL_SET_SEXT_INT(_R1, _R2)
#define RVMODEL_CLR_SEXT_INT(_R1, _R2)

#endif // _RVMODEL_MACROS_H
