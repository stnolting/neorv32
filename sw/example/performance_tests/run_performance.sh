#!/bin/bash
cd I
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32I_all clean_all exe
make sim
cd ..
cd M
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32M_all clean_all exe
make sim
cd ..
cd Zfinx
make USER_FLAGS+=-DRUN_CHECK USER_FLAGS+=-DUART0_SIM_MODE USER_FLAGS+=-DSILENT_MODE USER_FLAGS+=-Drv32Zfinx_all clean_all exe
make sim
cd ..

