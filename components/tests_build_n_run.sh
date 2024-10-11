#!/bin/sh

#VERBOSE=x
set -eu"$VERBOSE"

# clean
rm -rf test/build

# build
cmake -S test -B test/build
cmake --build test/build

# run
./test/build/ut_uwb
