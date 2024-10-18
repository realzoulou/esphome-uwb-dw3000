#!/bin/sh

#VERBOSE=x
set -eu"$VERBOSE"

# clean
rm -rf test/build

# build
cmake -S test -B test/build
cmake --build test/build

# run all tests
./test/build/ut_uwb

# or run a specific test
# ./test/build/ut_uwb --gtest_filter=Location_calculatePosition.threeAnchors_real
