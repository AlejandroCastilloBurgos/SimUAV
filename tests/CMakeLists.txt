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
)

target_link_libraries(simuav_tests PRIVATE
    simuav_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(simuav_tests)
