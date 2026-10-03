#!/bin/sh
# Builds the sketch against the host mocks and runs the scenario tests. Needs g++ (any recent version).
set -e
cd "$(dirname "$0")"
mkdir -p build
g++ -std=gnu++11 -O1 -g -fpermissive -fno-omit-frame-pointer -fsanitize=address,undefined \
    -Wall -Wno-unused-function -Wno-unused-variable -Wno-format \
    -I mocks -o build/hostsim test.cpp mocks/mocks.cpp
./build/hostsim
