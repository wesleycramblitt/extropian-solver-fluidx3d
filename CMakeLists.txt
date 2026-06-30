cmake_minimum_required(VERSION 3.21)
project(extropian-solver-fluidx3d VERSION 0.1.0 LANGUAGES CXX
    DESCRIPTION "FluidX3D LBM solver plugin for exd::physics::ISolverPlugin"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies: use FetchContent when not already available ──
include(FetchContent)

if(NOT TARGET exd::core)
    FetchContent_Declare(exd-core
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-core.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-core)
endif()

if(NOT TARGET exd::physics)
    FetchContent_Declare(exd-physics
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-physics.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-physics)
endif()

if(NOT TARGET exd::render)
    FetchContent_Declare(exd-render
        GIT_REPOSITORY https://github.com/wesleycramblitt/extropian-render.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(exd-render)
endif()
find_package(OpenCL REQUIRED)

# FluidX3D-CLI: set FLUIDX3D_SOURCE_DIR to your local checkout path
set(FLUIDX3D_SOURCE_DIR "" CACHE PATH "Path to FluidX3D-CLI source directory")
if(FLUIDX3D_SOURCE_DIR)
    add_subdirectory(${FLUIDX3D_SOURCE_DIR} fluidx3d_build)
endif()

add_library(exd-solver-fluidx3d SHARED
    src/fluidx3d_plugin.cpp
    src/fluidx3d_system.cpp
)

target_include_directories(exd-solver-fluidx3d PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(exd-solver-fluidx3d PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(exd-solver-fluidx3d PUBLIC
    exd::core
    exd::physics
    exd::render
    OpenCL::OpenCL
)

if(TARGET fluidx3d)
    target_link_libraries(exd-solver-fluidx3d PRIVATE fluidx3d)
endif()

add_library(exd::solver::fluidx3d ALIAS exd-solver-fluidx3d)

option(EXT_SOLVER_FLUIDX3D_BUILD_TESTS "Build tests" OFF)
if(EXT_SOLVER_FLUIDX3D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
