# Build and run development Docker container.
#
# To build and run the osrm-backend server, run:
#   $ ./bks/start_osrm.sh
# Then the server can be accessed like:
#   http://localhost:5004/route/v1/bike/-104.98307,39.69118;-104.999394,39.773106?overview=full&geometries=polyline&steps=true&annotations=true&alternatives=3

set -e

if [ "$#" -ne 1 ]; then
  echo "Usage: $0 <path to routing repo>"
  exit 1
fi

DOCKER_TAG=osrm-backend-dev

docker build . -t $DOCKER_TAG -f docker/Dockerfile-dev

docker run --rm -it \
  -v $(pwd):/mnt/src \
  -e CCACHE_DIR=/mnt/src/.ccache \
  -v $1/osrm/data:/data \
  -p 5004:5000 \
  --entrypoint "bash" \
  $DOCKER_TAG:latest
