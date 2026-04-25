# URBANA BOARD CONSTRAINTS FOR NEORV32 SoC

# Set Bank 0 voltage
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.Config.SPI_buswidth 4 [current_design]
set_property INTERNAL_VREF 0.675 [get_iobanks 34]
set_property BITSTREAM.CONFIG.UNUSEDPIN PULLNONE [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]

# -------------------------------------------------------------------------
# 1. CLOCK (1 Port) - Matches 'clk_100MHz'
# -------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN N15 IOSTANDARD LVCMOS33} [get_ports {clk_100MHz}]

# -------------------------------------------------------------------------
# 2. RESET (1 Port) - Matches 'reset_rtl_0'
# Mapped to the dedicated reset button on the board (B7)
# -------------------------------------------------------------------------
#set_property -dict {PACKAGE_PIN B7 IOSTANDARD LVCMOS25} [get_ports {reset_rtl_0}]
set_property -dict {PACKAGE_PIN J2 IOSTANDARD LVCMOS25} [get_ports {reset_rtl_0}]

# -------------------------------------------------------------------------
# 3. UART (2 Ports) - Matches 'uart0_txd_o_0' and 'uart0_rxd_i_0'
# -------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN A16 IOSTANDARD LVCMOS33} [get_ports {uart0_txd_o_0}]
set_property -dict {PACKAGE_PIN B16 IOSTANDARD LVCMOS33} [get_ports {uart0_rxd_i_0}]

# -------------------------------------------------------------------------
# 4. GPIO INPUTS (8 Ports) - Matches 'gpio_i_0[7:0]'
# Mapped to the first 8 slide switches on the board
# -------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN G1 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[0]}]
set_property -dict {PACKAGE_PIN F2 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[1]}]
set_property -dict {PACKAGE_PIN F1 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[2]}]
set_property -dict {PACKAGE_PIN E2 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[3]}]
set_property -dict {PACKAGE_PIN E1 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[4]}]
set_property -dict {PACKAGE_PIN D2 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[5]}]
set_property -dict {PACKAGE_PIN D1 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[6]}]
set_property -dict {PACKAGE_PIN C2 IOSTANDARD LVCMOS25}  [get_ports {gpio_i_0[7]}]

# -------------------------------------------------------------------------
# 5. GPIO OUTPUTS (8 Ports) - Matches 'gpio_o_0[7:0]'
# Mapped to the first 8 LEDs on the board
# -------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN C13 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[0]}]
set_property -dict {PACKAGE_PIN C14 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[1]}]
set_property -dict {PACKAGE_PIN D14 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[2]}]
set_property -dict {PACKAGE_PIN D15 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[3]}]
set_property -dict {PACKAGE_PIN D16 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[4]}]
set_property -dict {PACKAGE_PIN F18 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[5]}]
set_property -dict {PACKAGE_PIN E17 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[6]}]
set_property -dict {PACKAGE_PIN D17 IOSTANDARD LVCMOS33} [get_ports {gpio_o_0[7]}]