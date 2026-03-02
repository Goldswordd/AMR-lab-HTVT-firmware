#!/bin/bash
# Download STM32F1xx HAL Driver and CMSIS from ST GitHub
# Simple approach: full shallow clone → copy needed directories
set -e

REPO_URL="https://github.com/STMicroelectronics/STM32CubeF1.git"
TMPDIR=$(mktemp -d)
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEST="$SCRIPT_DIR/Drivers"

echo "=== Cloning STM32CubeF1 (shallow, depth=1)... ==="
git clone --depth=1 "$REPO_URL" "$TMPDIR/stm32cube"

echo "=== Copying HAL Driver and CMSIS to project ==="
rm -rf "$DEST"
mkdir -p "$DEST"

cp -r "$TMPDIR/stm32cube/Drivers/STM32F1xx_HAL_Driver" "$DEST/"
cp -r "$TMPDIR/stm32cube/Drivers/CMSIS" "$DEST/"

echo "=== Cleanup ==="
rm -rf "$TMPDIR"

echo "=== Done! ==="
echo "HAL Driver:"
ls "$DEST/STM32F1xx_HAL_Driver/"
echo ""
echo "CMSIS:"
ls "$DEST/CMSIS/"
