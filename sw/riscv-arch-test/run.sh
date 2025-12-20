#!/usr/bin/env bash

set -e
cd $(dirname "$0")

# run riscof
riscof run \
--config=config.ini \
--suite=riscv-arch-test/riscv-test-suite/ \
--env=riscv-arch-test/riscv-test-suite/env \
--no-browser

# check report - run successful?
if grep -rniq riscof_work/report.html -e '>0failed<'
then
  echo "Test successful!"
  exit 0
else
  echo "Test FAILED!"
  exit 1
fi
