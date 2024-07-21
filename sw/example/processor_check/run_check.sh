#!/usr/bin/env bash

set -e

echo "Starting processor check simulation..."
make clean_all all sim-check
