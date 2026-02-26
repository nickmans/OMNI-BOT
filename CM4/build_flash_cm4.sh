#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEBUG_DIR="$SCRIPT_DIR/Debug"
ELF="$DEBUG_DIR/OMNI-BOT_CM4.elf"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$DEBUG_DIR"

find "$DEBUG_DIR" -type f \( -name 'makefile' -o -name '*.mk' -o -name '*.d' \) -print0 | \
  xargs -0 sed -i "s#C:/Users/Loren/Documents/H7code/OMNI-BOT#$PROJECT_ROOT#g"

find "$DEBUG_DIR" -type f \( -name 'makefile' -o -name '*.mk' -o -name '*.d' \) -print0 | \
  xargs -0 sed -i "s#C:\\\\Users\\\\Loren\\\\Documents\\\\H7code\\\\OMNI-BOT#$PROJECT_ROOT#g"

sed -i "s#$PROJECT_ROOT\\\\#$PROJECT_ROOT/#g" "$DEBUG_DIR/makefile"
sed -i 's#/CM4\\STM32H755ZITX_FLASH.ld#/CM4/STM32H755ZITX_FLASH.ld#g' "$DEBUG_DIR/makefile"

find "$DEBUG_DIR" -type f \( -name 'makefile' -o -name '*.mk' \) -print0 | \
  xargs -0 sed -i 's/ -fcyclomatic-complexity//g'

find "$DEBUG_DIR" -name '*.d' -type f -delete

make all -j"$(nproc)"

openocd \
  -f interface/stlink.cfg \
  -f target/stm32h7x.cfg \
  -c "program $ELF reset exit"
