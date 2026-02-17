cmake_minimum_required(VERSION 3.21)
project(imuSensor)

set(CMAKE_CXX_STANDARD 17)

# Set the SimpleBLE installation directory relative to this CMakeLists.txt file
set(SimpleBLE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/SimpleBLE-main/install_simpleble")

# Find SimpleBLE package
find_package(simpleble REQUIRED CONFIG 
    PATHS "${SimpleBLE_DIR}/lib/cmake/simpleble"
    NO_DEFAULT_PATH
)

# Add your source files
add_executable(imu_program main.cpp IMU.cpp)

# Link SimpleBLE to your target
target_link_libraries(imu_program PRIVATE simpleble::simpleble)

# Include directories
target_include_directories(imu_program PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SimpleBLE_DIR}/include
)