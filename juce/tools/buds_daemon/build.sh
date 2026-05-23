#!/bin/sh
set -e
cd "$(dirname "$0")"
clang++ -std=c++17 -fobjc-arc -O2 -Wall \
    -framework Foundation -framework IOBluetooth \
    main.mm -o buds_daemon
echo "built: $(pwd)/buds_daemon"
