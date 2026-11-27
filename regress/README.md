# Regression tests: did the hardware start behaving differently?

## The idea, explained simply

Think of a hardware block as a music box.

1. **We turn the handle the same way every time.** Each test feeds a block the same inputs, in the same
   order, on every run.
2. **We write down every note it plays.** On every clock tick, the test writes down the value of every
   output of the block.
3. **The first time, we keep that as "how the song goes".** That record is the *baseline*. It lives in
   [`golden/`](golden).
4. **Every later time, we compare.** If the song is the same, the test says nothing. If one note is
   different, it tells you which test, which output, and roughly at which clock tick.

That is all it does. It does not know what is *right*. It only knows **same** or **different**. You decide
whether a difference is the change you wanted or a mistake.

Editing comments or formatting never changes the result. Only behaviour does.

## What happens when I push?

The [`Regression`](../.github/workflows/Regression.yml) workflow runs when a push or pull request touches
`rtl/`, `sim/` or `regress/`. It takes a few minutes.

**It never fails your push.** The check is always green. If behaviour changed, you get a yellow warning on
the run and a summary that lists what changed. If you ignore it, nothing breaks. The same list simply
shows up again next time.

## It says something changed. What now?

Open the run summary. It looks like this:

```text
Changed: uart_fifo4
Test file: regress/unit/uart.vhdt
- first difference at about cycle 4096 (within 16 cycles)
- signal uart_txd_o first differs in cycles 4096..4351 (changes 812 -> 790)
```

Read it as: *in the test `uart_fifo4`, the output `uart_txd_o` did something different, starting around
clock tick 4096.* Then ask one question: **did I mean to change that?**

**Yes, I meant to.** Accept the new behaviour as the new baseline. Pick one way:

- *Without installing anything:* download the `regress-report` artifact from the workflow run. Copy its
  `golden` folder over `regress/golden` and commit.
- *On your machine:* run `python3 regress/run.py --update` and commit `regress/golden`.

**No, I did not.** The test found a bug for you. The summary names the output and the clock tick, which is
where to start looking.

**The summary says the tests could not be built.** A port or a generic of a tested block was renamed, added
or removed. Open the block's test file in [`unit/`](unit) and make the same change there. The message names
the file.

## Run it yourself

You need Ubuntu 24.04 (WSL works) with `gcc`, `make` and `python3`.

```bash
bash regress/tools.sh install
eval "$(bash regress/tools.sh env)"
python3 regress/run.py
```

The first line downloads one fixed GHDL version into `~/tools`. It needs no root. The workflow uses the
same script, so your machine and GitHub give the same result. A full run takes about one minute on four cores.

| Option | What it does |
|---|---|
| `--filter uart` | Run only the tests whose name matches. |
| `--list` | Show all test names. |
| `--update` | Accept the current behaviour as the new baseline. |
| `--keep-traces` | Keep the full record of every output under `<build>/run/<test>/trace.txt`. |
| `--build DIR` | Where to build. The default is `~/.cache/neorv32-regress/build`. |

## What is in this folder?

| Path | What it is |
|---|---|
| [`unit/`](unit) | The tests. One file per hardware block, for example `uart.vhdt` for `neorv32_uart`. |
| [`golden/`](golden) | The baselines. One file per test file, for example `uart.sig`. Never edit these by hand. |
| `run.py` | Runs everything and compares. |
| `tbgen.py` | Turns a test file into a complete VHDL testbench. |
| `lib/` | A few shared VHDL helpers: a random-number source, a simple bus memory, an AXI device. |
| `tools.sh` | Installs the one fixed GHDL version. |

There are 164 tests in 59 test files. They cover the blocks that can run on their own: the
peripherals, the bus blocks, the caches, the debug blocks and the CPU's building blocks. They need only
GHDL. No RISC-V compiler and no software is involved.

## What does a test file look like?

This is all of [`unit/sys_clock.vhdt`](unit/sys_clock.vhdt):

```vhdl
--! dut neorv32_sys_clock
--! stim
  tick(10000);
```

It says: *test the block `neorv32_sys_clock`, and let it run for 10000 clock ticks.* Everything else is
added for you: the clock, the reset, a signal for every port, and the code that writes down every output.
You never list the outputs yourself, so a new output is recorded automatically.

A longer example is [`unit/gpio.vhdt`](unit/gpio.vhdt). The lines that start with `--!` mean:

| Line | Meaning |
|---|---|
| `--! dut NAME` | The block to test. |
| `--! generic NAME=VALUE` | The default value of one of the block's generics. |
| `--! config NAME ...` | One test. Give it other generic values to test another configuration. |
| `--! decl` | Extra signals follow. |
| `--! arch` | Extra VHDL around the block follows, for example a loop-back wire. |
| `--! stim` | The inputs follow, written as ordinary VHDL. |

Inside `--! stim` you can use `tick(n)` to wait, and `wr(address, data)` and `rd(address)` for blocks with a
bus port. `randv(n)` gives `n` random bits, and they are the same random bits on every run.

To add a test, create a file in `unit/`, then run `python3 regress/run.py --filter NAME --update` and commit
the new baseline with it.

## Questions

**Do I have to do anything when I add a new block?** No. A block without a test file is not checked.

**Does a version bump or a software change trigger it?** No. These tests use no software and do not see the
version number.

**Why a fixed GHDL version, and not the one the other workflows use?** So that a baseline recorded on one
machine matches on every other. `tools.sh` checks the download against a fixed checksum.

**How do I know the tests notice real changes?** Change one line in a block, for example invert a signal,
and run its test. It reports the output and the clock tick. Undo the change and it is quiet again.

**What is in a baseline file?** Not the full record, which would be far too large. For each output it holds
a checksum, the number of value changes, and checksums of up to 32 slices of time. That is enough to say
which output changed and when.
