cmake_minimum_required(VERSION 3.21)
project(extropian-solver-fluidx3d VERSION 0.1.0 LANGUAGES CXX
    DESCRIPTION "FluidX3D LBM solver plugin implementing ext::physics::ISolverPlugin"
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Dependencies ──────────────────────────────────────
find_package(ext-core REQUIRED)
find_package(ext-physics REQUIRED)
find_package(OpenCL REQUIRED)

# FluidX3D shared library (from your local fork)
# Adjust FLUIDX3D_SOURCE_DIR to point to your FluidX3D-CLI checkout
set(FLUIDX3D_SOURCE_DIR "" CACHE PATH "Path to FluidX3D-CLI source directory")
if(FLUIDX3D_SOURCE_DIR)
    add_subdirectory(${FLUIDX3D_SOURCE_DIR} fluidx3d_build)
else()
    message(WARNING "FLUIDX3D_SOURCE_DIR not set. FluidX3D solver will not link correctly.")
endif()

# ── Plugin library ────────────────────────────────────
add_library(ext-solver-fluidx3d SHARED
    src/fluidx3d_plugin.cpp
    src/fluidx3d_system.cpp
)

target_include_directories(ext-solver-fluidx3d PUBLIC include)
target_include_directories(ext-solver-fluidx3d PRIVATE src)

target_link_libraries(ext-solver-fluidx3d PUBLIC
    ext::core
    ext::physics
    OpenCL::OpenCL
)

if(TARGET fluidx3d)
    target_link_libraries(ext-solver-fluidx3d PRIVATE fluidx3d)
endif()

# Export the ISolverPlugin factory symbols
target_compile_definitions(ext-solver-fluidx3d PRIVATE SOLVER_PLUGIN_EXPORT)

add_library(ext::solver::fluidx3d ALIAS ext-solver-fluidx3d)

# ── Tests ─────────────────────────────────────────────
option(EXT_SOLVER_FLUIDX3D_BUILD_TESTS "Build tests" OFF)
if(EXT_SOLVER_FLUIDX3D_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
