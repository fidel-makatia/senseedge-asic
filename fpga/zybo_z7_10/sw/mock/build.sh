#!/bin/sh
# Build the board program against the mock chip for host-side protocol tests: build/mock_fw.
# On macOS a broken SDK can fail at link time; try the default toolchain, then each SDK.
cd "$(dirname "$0")/../.." && mkdir -p build
SRC="sw/senseedge_zybo.c sw/mock/mock_chip.c"
FLAGS="-std=gnu99 -O1 -Wall -Wextra -I sw/mock -DSE_BAUD=115200"
cc $FLAGS -o build/mock_fw $SRC 2>/dev/null && { echo "built build/mock_fw"; exit 0; }
for sdk in /Library/Developer/CommandLineTools/SDKs/MacOSX2*.sdk; do
  cc -isysroot "$sdk" $FLAGS -o build/mock_fw $SRC 2>/dev/null && { echo "built build/mock_fw with $sdk"; exit 0; }
done
echo "mock build failed"; cc $FLAGS -o build/mock_fw $SRC; exit 1
