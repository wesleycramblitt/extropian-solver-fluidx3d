#pragma once

#include <cstdint>

namespace ext::solver::fluidx3d {

/// ECS component: domain grid dimensions.
struct SimulationDomain {
    int nx = 64, ny = 64, nz = 64;
};

/// ECS component: fluid physical properties.
struct FluidPhysics {
    float nu = 0.01f;                  // viscosity
    float streamwise_velocity = 0.05f; // inlet speed
    uint8_t streamwise_axis = 0;       // 0=X, 1=Y, 2=Z
    float fx = 0, fy = 0, fz = 0;     // volume force
    float sigma = 0;                   // surface tension (0 = single phase)
};

/// ECS component: solver-specific configuration.
struct FluidX3DSolverConfig {
    uint8_t velocity_set = 19;          // D3Q19
    uint8_t collision_model = 0;
    uint8_t precision = 0;              // 0=FP32, 1=FP16S, 2=FP16C
    uint32_t dx = 1, dy = 1, dz = 1;   // multi-GPU subdivisions
    uint32_t extensions = 0;            // bitmask
};

/// ECS component: runtime simulation state.
enum class SimulationStatus : uint8_t {
    Stopped,
    Running,
    Error
};

struct SimulationInfo {
    SimulationStatus status = SimulationStatus::Stopped;
    uint32_t current_step = 0;
};

/// ECS component: transform (position, rotation, scale).
struct Transform {
    float pos_x = 0, pos_y = 0, pos_z = 0;
    float rot_w = 1, rot_x = 0, rot_y = 0, rot_z = 0;
    float scale_x = 1, scale_y = 1, scale_z = 1;
    float skew_xy = 0, skew_xz = 0, skew_yz = 0;
};

/// ECS component: reference to parent simulation entity.
struct SimulationReference {
    uint32_t simulation_entity_id = 0;
};

/// ECS component: GPU mesh handle.
struct Renderable {
    uint32_t mesh_handle = 0;
};

/// ECS component: particle cloud data.
struct ParticleCloud {
    int max_particles = 100000;
};

/// ECS component: volume field data.
struct VolumeField {
    int nx = 0, ny = 0, nz = 0;
};

/// ECS component: mesh asset file path.
struct MeshAsset {
    std::string path;
};

/// ECS component: marks an entity as disabled.
struct Disabled {};

} // namespace ext::solver::fluidx3d
