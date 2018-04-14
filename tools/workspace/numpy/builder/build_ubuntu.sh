#!/bin/bash
set -e -u -x

# Creates and runs a Docker container which builds NumPy for Ubuntu 16.04.
# To be called on the host system.

cd $(dirname $0)

docker build -t numpy_builder .

mkdir -p build
cp ./build_in_docker.sh build
docker run --rm -v ${PWD}/build:/build numpy_builder \
    /build/build_in_docker.sh /build
