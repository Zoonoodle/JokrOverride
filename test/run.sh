#!/bin/sh
# Host-side tests for the pure parts of the lift and the claw. Nothing here is built or uploaded by pros make --
# the PROS makefile only globs $(ROOT)/src, so this directory is invisible to the robot build.
#
#   test/run.sh
#
# Needs only the Mac's own clang++ (Xcode command line tools), no PROS toolchain.
set -e
cd "$(dirname "$0")/.."
DIR="$(mktemp -d)"

clang++ -std=c++20 -O2 -Wall -Wextra -Werror -o "$DIR/contact_detector_test" test/contact_detector_test.cpp
"$DIR/contact_detector_test"

clang++ -std=c++20 -O2 -Wall -Wextra -Werror -o "$DIR/autoclamp_detector_test" test/autoclamp_detector_test.cpp
"$DIR/autoclamp_detector_test"
