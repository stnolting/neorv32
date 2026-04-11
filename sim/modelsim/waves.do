#==============================================================================
# File: waves.do
#
# Description:
#   Waveform configuration for ModelSim.
#   Signals are grouped logically. General viewer settings are applied at the end.
#==============================================================================

quietly WaveActivateNextPane {} 0

#===========================================================================
# CLOCK / RESET
#===========================================================================
add wave -divider "CLOCK / RESET" \
    -color white -radix binary sim:/tb_project_name/tb_clk \
    -color white -radix binary sim:/tb_project_name/tb_rst

#===========================================================================
# INPUTS
#===========================================================================
add wave -divider "INPUTS" \
    -color green -radix hex    sim:/tb_project_name/tb_data_i \
    -color green -radix binary sim:/tb_project_name/tb_valid_i

#===========================================================================
# OUTPUTS
#===========================================================================
add wave -divider "OUTPUTS" \
    -color green -radix hex    sim:/tb_project_name/tb_data_o \
    -color green -radix binary sim:/tb_project_name/tb_valid_o

#===========================================================================
# DUT INTERNAL SIGNALS
#===========================================================================
add wave -divider "DUT INTERNAL" \
    -color green    -radix symbolic sim:/tb_project_name/dut/state \
    -color green    -radix hex      sim:/tb_project_name/dut/data_reg \
    -color green    -radix binary   sim:/tb_project_name/dut/valid_reg

#==============================================================================
# GENERAL WAVEFORM VIEWER SETTINGS
#==============================================================================

# Column widths
configure wave -namecolwidth 200
configure wave -valuecolwidth 60
configure wave -signalnamewidth 1
# Timeline units
configure wave -timelineunits ns
# Restore initial zoom
WaveRestoreZoom {0 ns} {500 ns}


#==============================================================================
# END OF FILE
#==============================================================================