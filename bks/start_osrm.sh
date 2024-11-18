# Build and run the osrm-backend routing server.
#
# Intended to be ran from development Docker image:
#   $ ./bks/start_dev_env.sh

set -e

mkdir -p build
cd build

cmake .. -DCMAKE_BUILD_TYPE=Release -DENABLE_ASSERTIONS=Off -DBUILD_TOOLS=Off -DENABLE_LTO=On

make -j$(nproc) install

osrm-routed --verbosity "DEBUG" --algorithm mld "/data/processed.osrm"
