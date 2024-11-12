# Build and run the osrm-backend routing server.
#
# Intended to be ran from development Docker image:
#   $ ./bks/start_dev_env.sh

set -e

cd build

make -j$(nproc) install

osrm-routed --verbosity "DEBUG" --algorithm mld "/data/processed.osrm"
