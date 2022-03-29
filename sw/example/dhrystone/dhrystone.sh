#!/usr/bin/env bash

set -e

echo "Generating dhrystone executable..."
make USER_FLAGS+="-DRUN_DHRYSTONE -DDHRY_ITERS=2000000 -DNOENUM" MARCH=rv32imc EFFORT=-O3 clean_all exe
