// Zcmp instruction fetch-fault test
//
// Verifies that an instruction access fault on the 16-bit Zcmp instruction
// word itself is not swallowed by the Zcmp micro-op sequencer. The frontend
// must report the fetch fault (via the IPB fault bit) instead of expanding
// the (faulted) halfword into a micro-op sequence.
//
// Trigger: a small RAM buffer is filled with "cm.push {ra}, -16" followed by
// c.nop padding, then a locked PMP NAPOT region (R+W, no eXecute) is put
// over the buffer and we jump into it. The fetch succeeds on the bus but
// the PMP raises pmp_err -> the IPB entry carries the fault bit together
// with the real (Zcmp-looking) instruction data.
//
// Expected: exactly one instruction access fault with mepc = buffer start
// and no side effects (sp untouched).
// Buggy:    the cm.push executes (sp -= 16, stores below sp) and the fault
//           is only raised for the *following* c.nop (mepc = buffer + 2).
//
// The trap handler does not advance mepc (everything in the buffer faults);
// it redirects execution to a landing pad address stored in ff_resume.
//
// NOTE: this test locks a PMP entry (cannot be unlocked until reset), so it
// should run last.

#include <neorv32.h>
#include "cm_fetch_fault.h"

// ---------------------------------------------------------------------------
// Trap recording
// ---------------------------------------------------------------------------
static volatile uint32_t ff_mepc;
static volatile uint32_t ff_mcause;
static volatile uint32_t ff_count;
static volatile uint32_t ff_resume; // handler redirects mepc here

static uint32_t ff_stack[64] __attribute__((aligned(16)));

// 64-byte executable buffer, naturally aligned for a 64-byte NAPOT region
static uint32_t code_buf[16] __attribute__((aligned(64)));

// ---------------------------------------------------------------------------
// Minimal trap handler (naked)
//
// Records mepc/mcause, then redirects execution to *ff_resume (mepc is NOT
// advanced - re-executing anything inside the PMP-protected buffer would
// fault again).
// ---------------------------------------------------------------------------
static void __attribute__((naked, aligned(4))) zcmp_ff_handler(void)
{
  asm volatile(
      "csrrw sp, mscratch, sp    \n"
      "addi  sp, sp, -8          \n"
      "sw    a0, 0(sp)           \n"
      "sw    a1, 4(sp)           \n"
      "csrr  a0, mepc            \n"
      "la    a1, ff_mepc         \n"
      "sw    a0, 0(a1)           \n"
      "csrr  a0, mcause          \n"
      "la    a1, ff_mcause       \n"
      "sw    a0, 0(a1)           \n"
      "la    a1, ff_count        \n"
      "lw    a0, 0(a1)           \n"
      "addi  a0, a0, 1           \n"
      "sw    a0, 0(a1)           \n"
      // redirect resume address
      "la    a1, ff_resume       \n"
      "lw    a0, 0(a1)           \n"
      "csrw  mepc, a0            \n"
      "lw    a0, 0(sp)           \n"
      "lw    a1, 4(sp)           \n"
      "addi  sp, sp, 8           \n"
      "csrrw sp, mscratch, sp    \n"
      "mret                      \n");
}

static uint32_t saved_mtvec;

static void install_ff_handler(void)
{
  neorv32_cpu_csr_write(CSR_MSCRATCH, (uint32_t)&ff_stack[63]);
  saved_mtvec = neorv32_cpu_csr_read(CSR_MTVEC);
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&zcmp_ff_handler));
}

static void restore_rte_handler(void)
{
  neorv32_cpu_csr_write(CSR_MTVEC, saved_mtvec);
}

// ---------------------------------------------------------------------------
// entry point
// ---------------------------------------------------------------------------
int cm_fetch_fault(void)
{
  int result = 0;

  neorv32_uart0_printf("\n--- Zcmp fetch-fault test ---\n");

  if (neorv32_cpu_pmp_get_num_regions() == 0)
  {
    neorv32_uart0_printf("PMP not available, skipping.\n");
    return 0;
  }

  // build code: cm.push {ra}, -16 (0xb842) followed by c.nop padding
  code_buf[0] = 0x0001b842u; // [cm.push {ra},-16] [c.nop]
  for (int i = 1; i < 16; i++)
  {
    code_buf[i] = 0x00010001u; // [c.nop] [c.nop]
  }
  asm volatile("fence" ::: "memory"); // data sync
  asm volatile("fence.i");            // flush i-cache

  // locked (M-mode enforced) NAPOT 64-byte region: R+W, no eXecute
  uint32_t pmp_addr = (((uint32_t)&code_buf[0]) >> 2) | 0x7u; // NAPOT, 64 bytes
  uint8_t pmp_cfg = (1 << PMPCFG_L) | (PMP_NAPOT << PMPCFG_A_LSB) | (1 << PMPCFG_W) | (1 << PMPCFG_R);
  if (neorv32_cpu_pmp_configure_region(0, pmp_addr, pmp_cfg) != 0)
  {
    neorv32_uart0_printf("PMP region setup FAILED, skipping.\n");
    return 0;
  }

  install_ff_handler();

  ff_count = 0;
  ff_mepc = 0;
  ff_mcause = 0;

  neorv32_uart0_printf("cm.push fetch access-fault");

  uint32_t sp_before = 0;
  uint32_t sp_at_landing = 0;

  asm volatile(
      // set up resume (landing pad) address for the trap handler
      "la  a0, 9f                \n"
      "la  a1, ff_resume         \n"
      "sw  a0, 0(a1)             \n"
      // save sp and jump into the execute-protected buffer
      "mv  t1, sp                \n"
      "mv  %[spb], sp            \n"
      "jr  %[buf]                \n"
      "9:                        \n"
      // capture sp as seen at the landing pad, then restore it
      "mv  %[spl], sp            \n"
      "mv  sp, t1                \n"
      : [spb] "=&r"(sp_before), [spl] "=&r"(sp_at_landing)
      : [buf] "r"(&code_buf[0])
      : "a0", "a1", "t1", "memory");

  int ok = 1;

  if (ff_count != 1)
  {
    neorv32_uart0_printf("\n  trap count: expected 1, got %u", ff_count);
    ok = 0;
  }
  if (ff_mcause != TRAP_CODE_I_ACCESS)
  {
    neorv32_uart0_printf("\n  mcause: expected 0x%x, got 0x%x", (uint32_t)TRAP_CODE_I_ACCESS, ff_mcause);
    ok = 0;
  }
  if (ff_mepc != (uint32_t)&code_buf[0])
  {
    neorv32_uart0_printf("\n  mepc: expected 0x%x, got 0x%x", (uint32_t)&code_buf[0], ff_mepc);
    ok = 0;
  }
  if (sp_at_landing != sp_before) // sp must be unchanged (cm.push must NOT have executed)
  {
    neorv32_uart0_printf("\n  sp changed by %d (cm.push executed!)", (int32_t)(sp_at_landing - sp_before));
    ok = 0;
  }

  if (ok)
  {
    neorv32_uart0_printf(" - OK\n");
  }
  else
  {
    neorv32_uart0_printf(" - FAIL\n");
    result = 1;
  }

  restore_rte_handler();

  if (result == 0)
  {
    neorv32_uart0_printf("Fetch-fault test PASSED.\n");
  }

  return result;
}
