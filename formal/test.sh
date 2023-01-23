#!/usr/bin/env sh

set -e

MAKE=make

cd $(dirname "$0")

case "$1" in
  prove)
    true
  ;;
  *)
    echo "Unknown test suite '$1'"
    exit 1
  ;;
esac

run_task() {
  echo "::group::Test $1"
  cd "$1"
  $MAKE "$2"
  cd ../
  echo '::endgroup::'
}

for item in neorv32_gpio; do
  run_task "$item" "$1"
done