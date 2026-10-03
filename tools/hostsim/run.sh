#!/bin/sh
# Builds the sketch against the host mocks and runs the scenario tests. Needs g++ (any recent version).
set -e
cd "$(dirname "$0")"
mkdir -p build
g++ -std=gnu++11 -O1 -g -fpermissive -fno-omit-frame-pointer -fsanitize=address,undefined \
    -Wall -Wno-unused-function -Wno-unused-variable -Wno-format \
    -I mocks -o build/hostsim test.cpp mocks/mocks.cpp
./build/hostsim build/sim_serial.log
# Smoke test of the recon summarizer on the simulated device log (needs python3).
if command -v python3 >/dev/null 2>&1; then
  mkdir -p build/sim_recon
  cp build/sim_serial.log build/sim_recon/serial.log
  python3 ../../recon/tools/recon_capture.py --summarize build/sim_recon/serial.log --ecu hostsim-ecu-00 \
    && echo "recon summary: build/sim_recon/summary.md"
fi
