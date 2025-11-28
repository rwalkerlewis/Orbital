cmake_minimum_required(VERSION 3.20)
project(Orbital LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SRC_FILES
    src/main.cpp
    src/Atmosphere.cpp
    src/Rocket.cpp
    src/Simulation.cpp
    src/Visualization.cpp
)

add_executable(orbital ${SRC_FILES})
target_include_directories(orbital PRIVATE src)
