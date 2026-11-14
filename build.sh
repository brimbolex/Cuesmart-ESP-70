#!/bin/sh
set -eu
cd "$(dirname "$0")"
"${ARDUINO_CLI:-arduino-cli}" compile --fqbn esp32:esp32:esp32s3 --libraries "$PWD/libraries" --build-path "$PWD/.build" "$PWD/CueSmart-ESP-70"
