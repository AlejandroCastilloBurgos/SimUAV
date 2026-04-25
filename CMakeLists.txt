cmake_minimum_required(VERSION 3.16)
project(SimUAV VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# ------------- Dependencies -------------------------------------------------

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# MAVLink C headers (header-only, dialect: common).
# Bootstrap: git clone https://github.com/mavlink/c_library_v2.git third_party/mavlink-c-library
set(MAVLINK_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/third_party/mavlink-c-library)
if(NOT EXISTS "${MAVLINK_INCLUDE_DIR}/common/mavlink.h")
    message(FATAL_ERROR
        "MAVLink headers not found at ${MAVLINK_INCLUDE_DIR}.\n"
        "Run: git clone https://github.com/mavlink/c_library_v2.git "
        "${CMAKE_SOURCE_DIR}/third_party/mavlink-c-library")
endif()

# ------------- Core library -------------------------------------------------

add_library(simuav_lib STATIC
    src/physics/QuadrotorModel.cpp
    src/physics/WindModel.cpp
    src/sensors/IMU.cpp
    src/sensors/GPS.cpp
    src/sensors/Barometer.cpp
    src/sensors/Magnetometer.cpp
    src/comms/MAVLinkBridge.cpp
    src/logging/JSONLogger.cpp
    src/logging/ULogLogger.cpp
    src/Simulator.cpp
)

target_include_directories(simuav_lib PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${MAVLINK_INCLUDE_DIR}
)

target_link_libraries(simuav_lib PUBLIC  Eigen3::Eigen)
target_link_libraries(simuav_lib PRIVATE pthread)

# ------------- Executable ---------------------------------------------------

add_executable(simuav src/main.cpp)
target_link_libraries(simuav PRIVATE simuav_lib)

# ------------- Tests --------------------------------------------------------

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
