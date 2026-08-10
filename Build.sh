#!/bin/bash
set -e

cd ~/openpuck

rm -f \
  build/openpuck/OpenPuck.ino.hex \
  build/openpuck/OpenPuck.ino.uf2 \
  /data/Modern/OpenPuck/OpenPuck.ino.hex \
  /data/Modern/OpenPuck/OpenPuck.ino.uf2

make build \
  BUILD_PATH=build/cache/openpuck \
  OUTPUT_DIR=build/openpuck

./gen_uf2.sh \
  build/openpuck/OpenPuck.ino.hex \
  build/openpuck/OpenPuck.ino.uf2

cp \
  build/openpuck/OpenPuck.ino.hex \
  build/openpuck/OpenPuck.ino.uf2 \
  /data/Modern/OpenPuck/

echo "Build Finished! HEX + UF2 now in SMB Share: Modern/OpenPuck"