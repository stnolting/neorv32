echo "Starting processor check simulation..."
make USER_FLAGS+="-DRUN_CHECK -DUART0_SIM_MODE -DUART1_SIM_MODE -g" MARCH=rv32imac clean_all sim
