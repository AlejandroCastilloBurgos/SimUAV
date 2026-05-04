include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(simuav_tests
    test_physics.cpp
    test_sensors.cpp
    test_config.cpp
    test_bridge.cpp
    test_ulog.cpp
    test_replay.cpp
    test_simulator.cpp
    test_status.cpp
    test_frames.cpp
)

target_link_libraries(simuav_tests PRIVATE
    simuav_lib
    GTest::gtest_main
)

target_compile_definitions(simuav_tests PRIVATE
    SIMUAV_SOURCE_DIR="${CMAKE_SOURCE_DIR}"
)

include(GoogleTest)
gtest_discover_tests(simuav_tests)
