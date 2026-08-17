# ------------------------------------------------------------------------------
# AEB sensing node
#
# Build layout mirrors the architecture, one static library per layer:
#
#   aeb_common     (header only)  domain types and the wire protocol
#   aeb_lidar                      acquisition; the only consumer of the Slamtec SDK
#   aeb_perception                 obstacle detection
#   aeb_canbus                     SocketCAN braking output
#   aeb_app                        composition root
#   aeb_node       (executable)    entry point
#
# The dependency edges are declared explicitly and deliberately:
#
#   aeb_lidar      -> aeb_common
#   aeb_perception -> aeb_common
#   aeb_canbus     -> aeb_common
#   aeb_app        -> aeb_lidar, aeb_perception, aeb_canbus
#
# None of the safety-path libraries (lidar, perception, canbus) may link
# each other or any development infrastructure. A link error enforces this
# better than a code review comment.
# ------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

project(aeb_node
    VERSION 0.1.0
    DESCRIPTION "Autonomous Emergency Braking sensing node"
    LANGUAGES CXX)

# ------------------------------------------------------------------------------
# Global settings
# ------------------------------------------------------------------------------

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)          # ISO C++17, no GNU dialect.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# A single-configuration generator with no explicit build type produces an
# unoptimised binary, which on a Pi Zero W is the difference between keeping up
# with the sensor and dropping frames.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING
        "Build type (Debug, Release, RelWithDebInfo, MinSizeRel)" FORCE)
    message(STATUS "aeb_node: CMAKE_BUILD_TYPE not set, defaulting to RelWithDebInfo")
endif()

# Export compile_commands.json for clang-tidy and editor tooling on the Mac.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------

option(AEB_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" ON)
option(AEB_BUILD_TESTS "Build unit tests (Phase 2)" OFF)

set(AEB_RPLIDAR_SDK_DIR "" CACHE PATH
    "Root of the Slamtec RPLidar SDK checkout, i.e. the directory containing sdk/")

# ------------------------------------------------------------------------------
# Common compiler configuration
#
# Carried by an INTERFACE target rather than global flags, so that third-party
# code (the Slamtec SDK) is never built with our warning policy.
# ------------------------------------------------------------------------------

add_library(aeb_compiler_flags INTERFACE)

target_compile_options(aeb_compiler_flags INTERFACE
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow                 # A shadowed variable in braking logic is a defect.
    -Wconversion             # Silent narrowing of distances or timestamps.
    -Wsign-conversion
    -Wcast-qual
    -Wnon-virtual-dtor
    -Wold-style-cast
    -Woverloaded-virtual
    -Wdouble-promotion       # Accidental float->double in the hot path.
    -Wformat=2)

if(AEB_WARNINGS_AS_ERRORS)
    target_compile_options(aeb_compiler_flags INTERFACE -Werror)
endif()

find_package(Threads REQUIRED)

# ------------------------------------------------------------------------------
# Slamtec RPLidar SDK
#
# The SDK ships as a Makefile project, not a CMake package, so it is located by
# search and wrapped in an IMPORTED target. Wrapping it means exactly one place
# in the build knows where the vendor code lives.
#
# Expected layout after running `make` in the SDK checkout:
#   <sdk_root>/sdk/include/sl_lidar.h
#   <sdk_root>/output/Linux/Release/libsl_lidar_sdk.a
# ------------------------------------------------------------------------------

find_path(SLAMTEC_SDK_INCLUDE_DIR
    NAMES sl_lidar.h
    HINTS
        "${AEB_RPLIDAR_SDK_DIR}/sdk/include"
        "${AEB_RPLIDAR_SDK_DIR}/include"
        "$ENV{RPLIDAR_SDK_DIR}/sdk/include"
        /usr/local/include/rplidar
    DOC "Directory containing sl_lidar.h")

# The SDK's public headers include private ones from src/ (hal/*.h), so that
# directory must be on the include path as well.
find_path(SLAMTEC_SDK_SRC_DIR
    NAMES hal/types.h
    HINTS
        "${AEB_RPLIDAR_SDK_DIR}/sdk/src"
        "${AEB_RPLIDAR_SDK_DIR}/src"
        "$ENV{RPLIDAR_SDK_DIR}/sdk/src"
    DOC "Directory containing the SDK's internal headers")

find_library(SLAMTEC_SDK_LIBRARY
    NAMES sl_lidar_sdk rplidar_sdk
    HINTS
        "${AEB_RPLIDAR_SDK_DIR}/output/Linux/Release"
        "${AEB_RPLIDAR_SDK_DIR}/output/Linux/Debug"
        "${AEB_RPLIDAR_SDK_DIR}/sdk/output/Linux/Release"
        "$ENV{RPLIDAR_SDK_DIR}/output/Linux/Release"
        /usr/local/lib
    DOC "Path to libsl_lidar_sdk.a")

if(NOT SLAMTEC_SDK_INCLUDE_DIR OR NOT SLAMTEC_SDK_LIBRARY)
    message(FATAL_ERROR
        "Slamtec RPLidar SDK not found.\n"
        "Build the SDK, then configure with:\n"
        "  cmake -DAEB_RPLIDAR_SDK_DIR=/path/to/rplidar_sdk ..\n"
        "Looked for sl_lidar.h and libsl_lidar_sdk.a; found:\n"
        "  include = ${SLAMTEC_SDK_INCLUDE_DIR}\n"
        "  library = ${SLAMTEC_SDK_LIBRARY}")
endif()

add_library(Slamtec::LidarSdk STATIC IMPORTED)
set_target_properties(Slamtec::LidarSdk PROPERTIES
    IMPORTED_LOCATION "${SLAMTEC_SDK_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${SLAMTEC_SDK_INCLUDE_DIR}")

if(SLAMTEC_SDK_SRC_DIR)
    set_property(TARGET Slamtec::LidarSdk APPEND PROPERTY
        INTERFACE_INCLUDE_DIRECTORIES "${SLAMTEC_SDK_SRC_DIR}")
endif()

message(STATUS "aeb_node: Slamtec SDK library ${SLAMTEC_SDK_LIBRARY}")

# ------------------------------------------------------------------------------
# aeb_common - domain types and wire protocol (header only)
# ------------------------------------------------------------------------------

add_library(aeb_common INTERFACE)

target_include_directories(aeb_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(aeb_common INTERFACE aeb_compiler_flags)

# ------------------------------------------------------------------------------
# aeb_lidar - acquisition
#
# Slamtec::LidarSdk is linked PRIVATE on purpose: the SDK's include directories
# and symbols stop here and are not propagated to anything that links aeb_lidar.
# ------------------------------------------------------------------------------

add_library(aeb_lidar STATIC
    lidar/Lidar.cpp
    lidar/RpLidarDevice.cpp)

target_link_libraries(aeb_lidar
    PUBLIC  aeb_common Threads::Threads
    PRIVATE Slamtec::LidarSdk)

# ------------------------------------------------------------------------------
# aeb_canbus - SocketCAN braking output
# ------------------------------------------------------------------------------

add_library(aeb_canbus STATIC
    canbus/CanBus.cpp)

target_link_libraries(aeb_canbus
    PUBLIC aeb_common)

# ------------------------------------------------------------------------------
# aeb_perception - obstacle detection
#
# Depends only on aeb_common, exactly like aeb_lidar, so it cannot acquire a
# dependency on the development networking layer.
# ------------------------------------------------------------------------------

add_library(aeb_perception STATIC
    perception/Perception.cpp)

target_link_libraries(aeb_perception
    PUBLIC aeb_common)

# ------------------------------------------------------------------------------
# aeb_app - composition root
# ------------------------------------------------------------------------------

add_library(aeb_app STATIC
    app/Application.cpp
    app/CommandLine.cpp
    app/SignalWaiter.cpp)

target_link_libraries(aeb_app
    PUBLIC aeb_lidar aeb_perception aeb_canbus)

# ------------------------------------------------------------------------------
# aeb_node - executable
# ------------------------------------------------------------------------------

add_executable(aeb_node app/main.cpp)

target_link_libraries(aeb_node PRIVATE aeb_app)

set_target_properties(aeb_node PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")

# ------------------------------------------------------------------------------
# Tests (Phase 2)
#
# The protocol, the frame queue and the command line parser are all pure logic
# with no hardware or socket dependency, which is precisely why they were
# separated into their own units.
# ------------------------------------------------------------------------------

if(AEB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------

include(GNUInstallDirs)

install(TARGETS aeb_node
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

message(STATUS "aeb_node: build type ${CMAKE_BUILD_TYPE}")
