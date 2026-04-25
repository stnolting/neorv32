#!/bin/bash
cd I
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all clean_all exe
make sim
cd ..
cd M
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all clean_all exe
make sim
cd ..
cd Zfinx
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32_all clean_all exe
make sim
cd ..

