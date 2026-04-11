# SEU-Resilient NEORV32 RISC-V Core (Student Research Project)

This project is a student research initiative at **ISAE SUPAERO** focused on improving the fault tolerance of a RISC-V soft-core processor by implementing **Single Event Upset (SEU) mitigation techniques**.

The base processor used in this work is the **NEORV32 RISC-V CPU** (From), which is being extended and modified at the RTL level to study and demonstrate hardware-level reliability improvements.

---

## Project Goals

The main objective of this project is to investigate and implement architectural and circuit-level mitigation techniques against SEUs, including (but not limited to):

- Redundancy based techniques (TMR)
- Error detection and correction mechanisms for memory and caches (EDAC)
- Parity bits on registers
- Fault detection strategies (Watchdog)

This work is part of a broader academic effort to study reliability in digital systems under radiation induced faults.

---

## Repository Structure

RTL modules of the mitigation techniques are placed under: rtl/seu
Testbench simulation files of the mitigation techniques are placed under: sim/tb
ModelSim related files for the simulation of the mitigation techniques are placed under: sim/modelsim
Vivado projects and files are placed under: /build
Constraint files for specific FPGAs are placed under: /constraints
Scripts for the generation of Vivado projects or the stimulation of AXI based tests are placed under: scripts

## Authors
- Aldo Lupio
- Olivier Oribes
- Teresa Bäurle

This is an open-source project that is free of charge and provided under an
permissive [license](https://github.com/stnolting/neorv32/blob/main/LICENSE).
See the [legal](https://stnolting.github.io/neorv32/#_legal) section for more information.
