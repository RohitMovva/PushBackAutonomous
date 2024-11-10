cmake_minimum_required(VERSION 3.10)
project(odometry_test)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find GTest package
find_package(GTest REQUIRED)

# Add test executable
add_executable(odometry_test 
    test_odometry.cpp
    src/odometry.cpp
)

# Link against GTest and pthread
target_link_libraries(odometry_test
    GTest::GTest
    GTest::Main
    pthread
)

# Include directories
target_include_directories(odometry_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)