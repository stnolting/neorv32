vlib modelsim_lib/work
vlib modelsim_lib/msim

vlib modelsim_lib/msim/xpm
vlib modelsim_lib/msim/xil_defaultlib
vlib modelsim_lib/msim/proc_sys_reset_v5_0_17
vlib modelsim_lib/msim/neorv32

vmap xpm modelsim_lib/msim/xpm
vmap xil_defaultlib modelsim_lib/msim/xil_defaultlib
vmap proc_sys_reset_v5_0_17 modelsim_lib/msim/proc_sys_reset_v5_0_17
vmap neorv32 modelsim_lib/msim/neorv32

vlog -work xpm  -incr -mfcu  -sv "+incdir+../../../../final.gen/sources_1/bd/design_1/ipshared/a9be" "+incdir+../../../../../../../2025.1/Vivado/data/rsb/busdef" \
"C:/2025.1/Vivado/data/ip/xpm/xpm_cdc/hdl/xpm_cdc.sv" \

vcom -work xpm  -93  \
"C:/2025.1/Vivado/data/ip/xpm/xpm_VCOMP.vhd" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../final.gen/sources_1/bd/design_1/ipshared/a9be" "+incdir+../../../../../../../2025.1/Vivado/data/rsb/busdef" \
"../../../bd/design_1/ip/design_1_clk_wiz_0_0/design_1_clk_wiz_0_0_clk_wiz.v" \
"../../../bd/design_1/ip/design_1_clk_wiz_0_0/design_1_clk_wiz_0_0.v" \

vcom -work proc_sys_reset_v5_0_17  -93  \
"../../../../final.gen/sources_1/bd/design_1/ipshared/9438/hdl/proc_sys_reset_v5_0_vh_rfs.vhd" \

vcom -work xil_defaultlib  -93  \
"../../../bd/design_1/ip/design_1_proc_sys_reset_0_0/sim/design_1_proc_sys_reset_0_0.vhd" \

vlog -work xil_defaultlib  -incr -mfcu  "+incdir+../../../../final.gen/sources_1/bd/design_1/ipshared/a9be" "+incdir+../../../../../../../2025.1/Vivado/data/rsb/busdef" \
"../../../bd/design_1/sim/design_1.v" \

vcom -work neorv32  -93  \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_package.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_bootrom.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_bootrom_image.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_bootrom_rom.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_bus.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_prim.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cache.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cache_ram.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cfs.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_clint.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_decompressor.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_frontend.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_control.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_hwtrig.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_counters.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_regfile.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_shifter.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_muldiv.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_bitmanip.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_fpu.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_cfu.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_cond.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu_crypto.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_alu.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_lsu.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_pmp.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu_trace.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_cpu.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_debug_auth.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_debug_dm.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_debug_dtm.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_dma.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_dmem.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_dmem_ram.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_gpio.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_gptmr.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_imem.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_imem_image.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_imem_ram.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_imem_rom.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_neoled.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_onewire.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_pwm.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_sdi.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_slink.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_spi.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_sys.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_sysinfo.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_xbus.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_wdt.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_uart.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_twi.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_twd.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_trng.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_tracer.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32/neorv32_top.vhd" \

vcom -work xil_defaultlib  -93  \
"../../../bd/design_1/ipshared/9444/src/xbus2axi4_bridge.vhd" \
"../../../bd/design_1/ipshared/9444/src/neorv32_vivado_ip.vhd" \
"../../../bd/design_1/ip/design_1_neorv32_vivado_ip_0_2/sim/design_1_neorv32_vivado_ip_0_2.vhd" \

vlog -work xil_defaultlib \
"glbl.v"

