make build BUILD_PATH=build/cache/openpuck OUTPUT_DIR=build/openpuck
./gen_uf2.sh build/openpuck/OpenPuck.ino.hex build/openpuck/OpenPuck.ino.uf2
echo "Build Finished! uf2 in ~/openpuck/build/openpuck"

