#!/bin/sh

#VERBOSE=x
set -eu"$VERBOSE"

BUILD_DIR=./build

# clean
rm -rf ${BUILD_DIR}

# build
cmake -S . -B ${BUILD_DIR}
cmake --build ${BUILD_DIR} --parallel 8

# run all tests
${BUILD_DIR}/ut_uwb

# or run specific test(s)
# ${BUILD_DIR}/ut_uwb --gtest_filter=Location_calculatePosition.threeAnchors_real
# ${BUILD_DIR}/ut_uwb --gtest_filter=UwbMessage.*
