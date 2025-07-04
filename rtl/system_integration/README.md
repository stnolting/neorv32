## Processor System Integration

### > `neorv32_litex_core_complex.vhd`

Pre-configured top entity wrapper for integration within the [LiteX](https://github.com/enjoy-digital/litex)
SoC builder framework. This wrapper provides AXI4 and AXI4-Stream-compatible interfaces.

> [!TIP]
> See the user guide section [`core/mem`](https://stnolting.github.io/neorv32/ug/#_litex_soc_builder_support)
for more information.

> [!NOTE]
> The provided top entity wrapper can also be used for custom (AXI) setups outside of Vivado IP block designs.


### > `neorv32_vivado_ip.vhd`

Processor top entity with optional AXI4 and AXI4-Stream interfaces. Dedicated for integration as custom
IP block within AMD Vivado. Run the provided packaging script in the Vivado TCL shell to generate a NEORV32
IP block:

```tcl
source neorv32_vivado_ip.tcl
```

This wrapper uses the `xbus2axi4_bridge.vhd` to convert the processor's XBUS protocol into the AXI4 protocol.

> [!TIP]
> See the user guide's [UG: Packaging the Processor as Vivado IP Block](https://stnolting.github.io/neorv32/ug/#_packaging_the_processor_as_vivado_ip_block)
section for more information and step-by-step instructions for generating a NEORV32 IP module.

### > `neorv32_libero_ip.vhd`

Processor top entity with optional AXI4 and AXI4-Stream interfaces. Dedicated for integration as custom
IP block within Microchip Libero. Run the provided 'neorv32_libero_ip.tcl' script in Libero after 
importing the RTL to generate a NEORV32 HDL+ core.

This wrapper uses the `xbus2axi4_bridge.vhd` to convert the processor's XBUS protocol into the AXI4 protocol.

> [!NOTE]
> BOOT_ADDR_CUSTOM and OCD_JEDEC_ID are not currently supported in the HDL+ core configurator and should be 
set in the 'neorv32_libero_ip.vhd' top file when using the SmartDesign flow.
