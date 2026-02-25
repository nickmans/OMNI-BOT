#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEBUG_DIR="$SCRIPT_DIR/Debug"
ELF="$DEBUG_DIR/OMNI-BOT_CM7.elf"

cd "$DEBUG_DIR"
make all -j"$(nproc)"

openocd \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -c "program $ELF verify reset exit"
