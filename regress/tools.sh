#!/usr/bin/env bash
# ================================================================================
# NEORV32 regression - pinned tool installer
# --------------------------------------------------------------------------------
# The single source of truth for the regression tool versions. The GitHub Action
# (.github/workflows/Regression.yml) and a local developer run both use this very
# script, so that both run the same binaries on the same OS release.
#
#   regress/tools.sh install   download, verify (SHA-256) and unpack the tools
#   regress/tools.sh env       print "export ..." lines, use: eval "$(regress/tools.sh env)"
#   regress/tools.sh check     verify the installed tools match the pins
#
# NEORV32_TOOLS selects the install directory (default: ~/tools).
# Nothing here needs root.
# ================================================================================

set -euo pipefail

TOOLS="${NEORV32_TOOLS:-$HOME/tools}"

# ---- pins ----------------------------------------------------------------------
# Changing any of these changes the compiled test programs and the coverage
# numbering, so re-run "regress/run.py --update" after changing a pin.
PIN_OS_ID="ubuntu"
PIN_OS_VER="24.04"

GHDL_VER="6.0.0"
GHDL_DIR="ghdl-gcc-${GHDL_VER}"
GHDL_URL="https://github.com/ghdl/ghdl/releases/download/v${GHDL_VER}/ghdl-gcc-${GHDL_VER}-ubuntu24.04-x86_64.tar.gz"
GHDL_SHA="58090ec736527e7cacd680031e70678a16887b7fb01e1c73498cfc952f2a71be"

GCC_VER="15.2.0-1"
GCC_DIR="xpack-riscv-none-elf-gcc-${GCC_VER}"
GCC_URL="https://github.com/xpack-dev-tools/riscv-none-elf-gcc-xpack/releases/download/v${GCC_VER}/xpack-riscv-none-elf-gcc-${GCC_VER}-linux-x64.tar.gz"
GCC_SHA="aaaa8060c914851a3e5ee1ba82cc3d6f80972f90638a05c6e823a37557a33758"

# Runtime library GHDL links against, missing from its archive. Unpacked from the
# Ubuntu release-pocket .deb, which stays in the archive for the life of 24.04.
GNAT_VER="13.2.0-23ubuntu4"
GNAT_DIR="libgnat-13-${GNAT_VER}"
GNAT_URL="https://archive.ubuntu.com/ubuntu/pool/universe/g/gcc-13/libgnat-13_${GNAT_VER}_amd64.deb"
GNAT_SHA="246d27d5539adc22b6a86c11fa7b6c32619e77aa08f054b15a6f3d1200dca375"
# Host packages that "ghdl -e" and gcov use. They come with the OS image, so they are
# not installed from here. A difference is reported as a warning, because it does not
# change simulation results as far as tested, but it is a difference from the baseline.
PIN_HOST="gcc-13=13.3.0-6ubuntu2~24.04.1 binutils=2.42-4ubuntu2.10 libc6-dev=2.39-0ubuntu8.9 zlib1g-dev=1:1.3.dfsg-3.1ubuntu2.3"
# --------------------------------------------------------------------------------

# The RISC-V compiler is only needed by the whole-processor tests, which run programs.
need_gcc() { [ -d "$(dirname "$0")/sw" ]; }

ghdl_bin() { echo "$TOOLS/$GHDL_DIR/bin"; }
gcc_bin()  { echo "$TOOLS/$GCC_DIR/bin"; }
gnat_lib() { echo "$TOOLS/$GNAT_DIR/usr/lib/x86_64-linux-gnu"; }

check_os() {
  local id ver
  id="$(. /etc/os-release && echo "$ID")"
  ver="$(. /etc/os-release && echo "$VERSION_ID")"
  if [ "$id" != "$PIN_OS_ID" ] || [ "$ver" != "$PIN_OS_VER" ]; then
    echo "ERROR: regression tools are pinned to $PIN_OS_ID $PIN_OS_VER, this is $id $ver" >&2
    return 1
  fi
}

fetch() { # url sha256 destination
  local url="$1" sha="$2" dst="$3"
  if [ -f "$dst" ] && echo "$sha  $dst" | sha256sum --check --status; then
    echo "already downloaded: $(basename "$dst")"
    return 0
  fi
  echo "downloading: $url"
  curl --fail --location --silent --show-error --retry 3 --output "$dst" "$url"
  echo "$sha  $dst" | sha256sum --check --quiet
}

cmd_install() {
  check_os
  mkdir -p "$TOOLS/.dl"

  if [ ! -d "$(gnat_lib)" ]; then
    fetch "$GNAT_URL" "$GNAT_SHA" "$TOOLS/.dl/$(basename "$GNAT_URL")"
    mkdir -p "$TOOLS/$GNAT_DIR"
    dpkg-deb -x "$TOOLS/.dl/$(basename "$GNAT_URL")" "$TOOLS/$GNAT_DIR"
  fi

  if [ ! -x "$(ghdl_bin)/ghdl" ]; then
    fetch "$GHDL_URL" "$GHDL_SHA" "$TOOLS/.dl/$(basename "$GHDL_URL")"
    mkdir -p "$TOOLS/$GHDL_DIR"
    tar -xzf "$TOOLS/.dl/$(basename "$GHDL_URL")" -C "$TOOLS/$GHDL_DIR"
    # the archive may hold a single top-level directory: flatten it
    if [ ! -d "$(ghdl_bin)" ]; then
      local top
      top="$(find "$TOOLS/$GHDL_DIR" -mindepth 2 -maxdepth 2 -type d -name bin | head -n 1)"
      [ -n "$top" ] || { echo "ERROR: unexpected GHDL archive layout" >&2; exit 1; }
      top="$(dirname "$top")"
      find "$top" -mindepth 1 -maxdepth 1 -exec mv -t "$TOOLS/$GHDL_DIR" {} +
      rmdir "$top"
    fi
  fi

  if need_gcc && [ ! -x "$(gcc_bin)/riscv-none-elf-gcc" ]; then
    fetch "$GCC_URL" "$GCC_SHA" "$TOOLS/.dl/$(basename "$GCC_URL")"
    tar -xzf "$TOOLS/.dl/$(basename "$GCC_URL")" -C "$TOOLS"
  fi

  cmd_check
}

cmd_env() {
  echo "export PATH=\"$(ghdl_bin):$(gcc_bin):\$PATH\""
  echo "export LD_LIBRARY_PATH=\"$(gnat_lib)\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\""
}

cmd_check() {
  check_os
  local rc=0
  local gv cv
  # "read" takes the first line. A pipe to "head" could close early and break the writer.
  read -r gv <<< "$(LD_LIBRARY_PATH="$(gnat_lib)${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$(ghdl_bin)/ghdl" --version)"
  echo "ghdl: $gv"
  case "$gv" in *"$GHDL_VER"*) ;; *) echo "ERROR: GHDL is not $GHDL_VER" >&2; rc=1 ;; esac
  if need_gcc; then
    read -r cv <<< "$("$(gcc_bin)/riscv-none-elf-gcc" --version)"
    echo "gcc : $cv"
    case "$cv" in *"${GCC_VER%-*}"*) ;; *) echo "ERROR: GCC is not $GCC_VER" >&2; rc=1 ;; esac
  fi
  local p want have
  for p in $PIN_HOST; do
    want="${p#*=}"
    p="${p%%=*}"
    have="$(dpkg-query -W -f='${Version}' "$p" 2>/dev/null || echo missing)"
    echo "host: $p $have"
    if [ "$have" != "$want" ]; then
      echo "WARNING: host package $p is $have, baselines were made with $want"
    fi
  done
  read -r have <<< "$(gcov --version)"
  echo "host: $have"
  return $rc
}

case "${1:-}" in
  install) cmd_install ;;
  env)     cmd_env ;;
  check)   cmd_check ;;
  *) echo "usage: $0 {install|env|check}" >&2; exit 2 ;;
esac
