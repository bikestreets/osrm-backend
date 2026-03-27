# Enable regeneration of the OSRM processed data from within the Docker image.

set -e

# Ensure the latest `osrm-*` tools are included.
cd build
make -j$(nproc) install

# Run the OSRM tools.
osrm-extract -p "../profiles/bikestreets.lua" "/data/processed.osm"
osrm-partition "/data/processed.osrm"
osrm-customize "/data/processed.osrm"
