# https://google.github.io/googletest/quickstart-cmake.html

cmake_minimum_required(VERSION 3.14)

project(ut_uwb CXX)

# GoogleTest requires at least C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Avoid warning about DOWNLOAD_EXTRACT_TIMESTAMP in CMake 3.24:
if (CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0")
  cmake_policy(SET CMP0135 NEW)
endif()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

set(UWB_SOURCE_DIR ../components/uwb)

add_executable(${PROJECT_NAME}
  AntDelayCalibrationTests.cpp
  LocationTests.cpp
  UwbAnchorDataTests.cpp
  UwbMessageTests.cpp
  ${UWB_SOURCE_DIR}/AntDelayCalibration.cpp
  ${UWB_SOURCE_DIR}/InitialMsg.cpp
  ${UWB_SOURCE_DIR}/FinalMsg.cpp
  ${UWB_SOURCE_DIR}/Location.cpp
  ${UWB_SOURCE_DIR}/ResponseMsg.cpp
  ${UWB_SOURCE_DIR}/UwbAnchorData.cpp
  ${UWB_SOURCE_DIR}/UwbMessage.cpp
)

target_link_libraries(${PROJECT_NAME}
  GTest::gtest_main
)

# sanitizers
set(SANITIZERS "")
string(APPEND SANITIZERS "-fsanitize=address ")
string(APPEND SANITIZERS "-fsanitize=pointer-subtract ")
string(APPEND SANITIZERS "-fsanitize=pointer-compare ")
string(APPEND SANITIZERS "-fsanitize=leak ")
string(APPEND SANITIZERS "-fsanitize=undefined ")
string(APPEND SANITIZERS "-fno-omit-frame-pointer ")
string(APPEND CMAKE_CXX_FLAGS "${SANITIZERS}")
string(APPEND CMAKE_LINKER_FLAGS "${SANITIZERS} -static-libasan")

include(GoogleTest)
gtest_discover_tests(${PROJECT_NAME})

# activate certain unit-test-only use-cases
add_compile_definitions(__UT_TEST__)

# set project specific include paths
include_directories(../components/uwb)

# enable verbose output if needed
set(CMAKE_VERBOSE_MAKEFILE ON)
