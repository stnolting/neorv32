// Zcmp async interrupt test
//
// Verifies that when a machine timer interrupt (MTI) fires during the
// non-atomic load/store micro-ops of a Zcmp cm.push sequence, mepc is
// set to the address of the Zcmp instruction itself (not Zcmp_PC + 2).
// On mret the micro-op sequence restarts from scratch, which is safe
// because the stores are idempotent (SP has not yet been adjusted).
//
// The same trap.pc code path is used for debug halt (dpc), so this test
// also validates correctness of dpc for debug requests during Zcmp.
//
// Approach: arm the machine timer with a tight deadline, then immediately
// execute cm.push {ra, s0-s11}, -64 (the largest push variant, 13 stores
// + 1 addi = widest non-atomic window). Sweep the timer offset to
// maximize the chance of the interrupt landing during micro-ops.

#include <neorv32.h>
#include "cm_irq.h"

// ---------------------------------------------------------------------------
// Trap recording
// ---------------------------------------------------------------------------
static volatile uint32_t irq_mepc;
static volatile uint32_t irq_mcause;
static volatile uint32_t irq_count;

static uint32_t irq_handler_stack[64] __attribute__((aligned(16)));

// ---------------------------------------------------------------------------
// Minimal trap handler (naked)
//
// Records mepc and mcause, disables MTI (so the restarted push is not
// interrupted again), and returns via mret WITHOUT advancing mepc.
// This causes the CPU to restart at mepc, which should be the cm.push
// instruction address.
// ---------------------------------------------------------------------------
static void __attribute__((naked, aligned(4))) zcmp_irq_handler(void)
{
  asm volatile(
      "csrrw sp, mscratch, sp    \n"
      "addi  sp, sp, -8          \n"
      "sw    a0, 0(sp)           \n"
      "sw    a1, 4(sp)           \n"
      // record mepc
      "csrr  a0, mepc            \n"
      "la    a1, irq_mepc        \n"
      "sw    a0, 0(a1)           \n"
      // record mcause
      "csrr  a0, mcause          \n"
      "la    a1, irq_mcause      \n"
      "sw    a0, 0(a1)           \n"
      // increment counter
      "la    a1, irq_count       \n"
      "lw    a0, 0(a1)           \n"
      "addi  a0, a0, 1           \n"
      "sw    a0, 0(a1)           \n"
      // disable MTI (clear mie.MTIE = bit 7)
      "li    a0, 0x80            \n"
      "csrc  mie, a0             \n"
      // restore and return — do NOT advance mepc
      "lw    a0, 0(sp)           \n"
      "lw    a1, 4(sp)           \n"
      "addi  sp, sp, 8           \n"
      "csrrw sp, mscratch, sp    \n"
      "mret                      \n");
}

// ---------------------------------------------------------------------------
// Trap handler install / restore
// ---------------------------------------------------------------------------
static uint32_t saved_mtvec;

static void install_irq_handler(void)
{
  neorv32_cpu_csr_write(CSR_MSCRATCH, (uint32_t)&irq_handler_stack[63]);
  saved_mtvec = neorv32_cpu_csr_read(CSR_MTVEC);
  neorv32_cpu_csr_write(CSR_MTVEC, (uint32_t)(&zcmp_irq_handler));
}

static void restore_rte_handler(void)
{
  neorv32_cpu_csr_write(CSR_MTVEC, saved_mtvec);
}

// ---------------------------------------------------------------------------
// Test frame for push verification
// ---------------------------------------------------------------------------
#define IRQ_FRAME_WORDS 64
static uint32_t irq_frame[IRQ_FRAME_WORDS] __attribute__((aligned(16)));
static volatile uint32_t irq_sp_save;

// Expected frame contents after cm.push {ra, s0-s11}, -64 completes on a
// 64-word frame. The push stores 13 registers (ra, s0..s11) at the top of
// the frame and adjusts SP by -64. The bottom 19 words are untouched.
// Register values: ra=0xa5000001, s0=0xa5000008, ..., s11=0xa500001b
static const uint32_t expected_frame[IRQ_FRAME_WORDS] = {
    0xdead0000,
    0xdead0001,
    0xdead0002,
    0xdead0003,
    0xdead0004,
    0xdead0005,
    0xdead0006,
    0xdead0007,
    0xdead0008,
    0xdead0009,
    0xdead000a,
    0xdead000b,
    0xdead000c,
    0xdead000d,
    0xdead000e,
    0xdead000f,
    0xdead0010,
    0xdead0011,
    0xdead0012,
    0xdead0013,
    0xdead0014,
    0xdead0015,
    0xdead0016,
    0xdead0017,
    0xdead0018,
    0xdead0019,
    0xdead001a,
    0xdead001b,
    0xdead001c,
    0xdead001d,
    0xdead001e,
    0xdead001f,
    0xdead0020,
    0xdead0021,
    0xdead0022,
    0xdead0023,
    0xdead0024,
    0xdead0025,
    0xdead0026,
    0xdead0027,
    0xdead0028,
    0xdead0029,
    0xdead002a,
    0xdead002b,
    0xdead002c,
    0xdead002d,
    0xdead002e,
    0xdead002f,
    0xdead0030,
    0xdead0031,
    0xdead0032,
    0xa5000001,
    0xa5000008,
    0xa5000009,
    0xa5000012,
    0xa5000013,
    0xa5000014,
    0xa5000015,
    0xa5000016,
    0xa5000017,
    0xa5000018,
    0xa5000019,
    0xa500001a,
    0xa500001b,
};

// ---------------------------------------------------------------------------
// Single trial: arm timer, execute cm.push, check results
//
// Returns: 0 = interrupt did not fire during push (inconclusive)
//          1 = interrupt fired during push and mepc + frame are correct
//         -1 = interrupt fired during push but mepc or frame is wrong
// ---------------------------------------------------------------------------
static int run_push_mti_trial(uint32_t timer_offset)
{
  volatile uint32_t *test_sp;
  uint32_t zcmp_pc = 0;
  uint32_t sp_after_push = 0;

  // Reset state
  irq_count = 0;
  irq_mepc = 0;
  irq_mcause = 0;

  // Fill frame with dead pattern
  for (int i = 0; i < IRQ_FRAME_WORDS; i++)
    irq_frame[i] = 0xdead0000 + i;

  test_sp = &irq_frame[IRQ_FRAME_WORDS];

  asm volatile(
      // === Phase 1: Setup (timer cannot fire yet) ===
      // Save all callee-saved registers on the real stack
      "addi sp, sp, -64          \n"
      "sw x1,   0(sp)            \n"
      "sw x8,   4(sp)            \n"
      "sw x9,   8(sp)            \n"
      "sw x18, 12(sp)            \n"
      "sw x19, 16(sp)            \n"
      "sw x20, 20(sp)            \n"
      "sw x21, 24(sp)            \n"
      "sw x22, 28(sp)            \n"
      "sw x23, 32(sp)            \n"
      "sw x24, 36(sp)            \n"
      "sw x25, 40(sp)            \n"
      "sw x26, 44(sp)            \n"
      "sw x27, 48(sp)            \n"
      // Save real sp and install test sp
      "la  a0, irq_sp_save       \n"
      "sw  sp, 0(a0)             \n"
      "mv  sp, %[tsp]            \n"
      // Load unique values into all pushable registers
      "li x1,  0xa5000001        \n"
      "li x8,  0xa5000008        \n"
      "li x9,  0xa5000009        \n"
      "li x18, 0xa5000012        \n"
      "li x19, 0xa5000013        \n"
      "li x20, 0xa5000014        \n"
      "li x21, 0xa5000015        \n"
      "li x22, 0xa5000016        \n"
      "li x23, 0xa5000017        \n"
      "li x24, 0xa5000018        \n"
      "li x25, 0xa5000019        \n"
      "li x26, 0xa500001a        \n"
      "li x27, 0xa500001b        \n"
      // Read MTIME and compute target
      // MTIME is at 0xFFF4BFF8; lui 0xFFF4C → 0xFFF4C000
      "lui a1, 0xfff4c           \n"
      "lw  a1, -8(a1)            \n" // a1 = MTIME low word
      "add a1, a1, %[ofs]        \n" // a1 = target
      // Prepare MTIMECMP[0] at 0xFFF44000 (prevent fire: hi = 0xFFFFFFFF)
      "lui a0, 0xfff44           \n" // a0 = &MTIMECMP[0] (kept for Phase 2)
      "li  t0, -1                \n"
      "sw  t0, 4(a0)             \n" // MTIMECMP[0].hi = 0xFFFFFFFF
      "sw  a1, 0(a0)             \n" // MTIMECMP[0].lo = target
      // Enable MTIE — safe: MTIMECMP.hi = 0xFFFFFFFF prevents fire
      "li  a1, 0x80              \n"
      "csrs mie, a1              \n"
      // Enable MIE — safe: MTIMECMP.hi = 0xFFFFFFFF prevents fire
      "li  a1, 0x08              \n"
      "csrs mstatus, a1          \n"
      // === Phase 2: ARM timer then immediately push ===
      // a0 still holds 0xFFF44000 = &MTIMECMP[0]
      "sw  zero, 4(a0)           \n" // MTIMECMP[0].hi = 0 → ARM!
      // cm.push {ra, s0-s11}, -64  (encoding 0xb8f2)
      // 13 stores + 1 addi sp = widest non-atomic window
      "1: .hword 0xb8f2          \n"
      // === Phase 3: Epilogue ===
      // Disable global interrupts
      "li  a0, 0x08              \n"
      "csrc mstatus, a0          \n"
      // Save resulting test SP before restoring machine state.
      "mv  t1, sp                \n"
      // Restore real sp and callee-saved registers
      "la  a0, irq_sp_save       \n"
      "lw  sp, 0(a0)             \n"
      "lw x1,   0(sp)            \n"
      "lw x8,   4(sp)            \n"
      "lw x9,   8(sp)            \n"
      "lw x18, 12(sp)            \n"
      "lw x19, 16(sp)            \n"
      "lw x20, 20(sp)            \n"
      "lw x21, 24(sp)            \n"
      "lw x22, 28(sp)            \n"
      "lw x23, 32(sp)            \n"
      "lw x24, 36(sp)            \n"
      "lw x25, 40(sp)            \n"
      "lw x26, 44(sp)            \n"
      "lw x27, 48(sp)            \n"
      "addi sp, sp, 64           \n"
      // Commit outputs at the end so they cannot be overwritten by restore code.
      "mv  %[sp_out], t1         \n"
      "la  %[pc], 1b             \n"
      : [sp_out] "=r"(sp_after_push), [pc] "=r"(zcmp_pc)
      : [tsp] "r"(test_sp), [ofs] "r"(timer_offset)
      : "a0", "a1", "t0", "t1", "memory");

  // Disable MTI in case it hasn't fired yet
  neorv32_cpu_csr_clr(CSR_MIE, 1 << CSR_MIE_MTIE);

  // Check: did the interrupt fire at all?
  if (irq_count == 0)
    return 0; // no interrupt — inconclusive

  // Check: did it fire during the cm.push?
  if (irq_mepc == (zcmp_pc + 2))
  {
    return -1; // bug signature: trap PC points past the interrupted Zcmp
  }

  if (irq_mepc != zcmp_pc)
  {
    return 0; // fired before or after the push — inconclusive
  }

  // The interrupt fired during the cm.push micro-ops.
  // Verify mcause
  if (irq_mcause != TRAP_CODE_MTI)
    return -1;

  // Verify SP adjustment: should be -64 from original test_sp
  uint32_t sp_expected = (uint32_t)&irq_frame[IRQ_FRAME_WORDS] - 64;
  if (sp_after_push != sp_expected)
    return -1;

  // Verify frame contents match expected
  for (int i = 0; i < IRQ_FRAME_WORDS; i++)
  {
    if (irq_frame[i] != expected_frame[i])
      return -1;
  }

  return 1; // success: interrupt during push, mepc correct, push completed correctly
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------
int cm_irq(void)
{
  int result = 0;
  int hits = 0;
  int failures = 0;

  neorv32_uart0_printf("\n--- Zcmp async interrupt (MTI) tests ---\n");

  if (!neorv32_clint_available())
  {
    neorv32_uart0_printf("CLINT not available, skipping.\n");
    return 0;
  }

  install_irq_handler();

  neorv32_uart0_printf("cm.push MTI mepc/restart");

  // Sweep timer offsets to maximize probability of hitting the micro-op window
  for (uint32_t offset = 1; offset <= 30; offset++)
  {
    int trial = run_push_mti_trial(offset);
    if (trial == 1)
    {
      hits++;
    }
    else if (trial == -1)
    {
      failures++;
    }
  }

  if (failures > 0)
  {
    neorv32_uart0_printf(" - FAIL (%d/%d bad)\n", failures, hits + failures);
    result = 1;
  }
  else if (hits == 0)
  {
    neorv32_uart0_printf(" - SKIP (no interrupt hit the push window)\n");
  }
  else
  {
    neorv32_uart0_printf(" - OK (%d hits)\n", hits);
  }

  restore_rte_handler();

  return result;
}
