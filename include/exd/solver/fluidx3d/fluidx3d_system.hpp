#pragma once

#include <exd/solver/fluidx3d/components.hpp>
#include <exd/ecs/registry.hpp>
#include <exd/render/graphics_context.hpp>
#include <exd/physics/solver/plugin_interface.hpp>

#include <vector>
#include <cstdint>
#include <future>
#include <memory>
#include <string>

// Forward: FluidX3D LBM type (external library)
class LBM;

namespace exd::solver::fluidx3d {

/// @brief ECS system that drives the FluidX3D LBM solver.
///
/// Each frame, iterates simulation entities in the registry and:
///   - Creates/destroys LBM solver instances as needed
///   - Detects config/transform changes → async solver rebuild
///   - Steps the solver, reads back particle data and volume field
///   - Writes particle data to ParticleCloud, volume to VolumeField components
///
/// Usage:
///   graph.add<FluidX3DSystem>(graphics_ctx).in_mode(SimMode::Simulate);
class FluidX3DSystem {
public:
    explicit FluidX3DSystem(exd::render::GraphicsContext& ctx);
    ~FluidX3DSystem();

    void update(exd::ecs::Registry& registry, float dt);

    /// Access the underlying solver (for direct coupling queries).
    LBM* solver() { return lbm_; }

private:
    void create_solver(exd::ecs::Registry& registry,
                       const SimulationDomain& domain,
                       const FluidPhysics& physics,
                       SimulationInfo& info,
                       const Transform& xform);

    void destroy_solver();
    void launch_async_rebuild(exd::ecs::Registry& registry,
                              const SimulationDomain& domain,
                              const FluidPhysics& physics,
                              SimulationInfo& info,
                              const Transform& xform);
    void check_async_rebuild(SimulationInfo& info);

    exd::render::GraphicsContext& ctx_;
    LBM* lbm_ = nullptr;

    std::vector<float> prev_particle_x_;
    struct { int nx = 0, ny = 0, nz = 0; } domain_cache_;

    // Solver cache: tracks the state the solver was built from
    struct SolverCache {
        int nx = 0, ny = 0, nz = 0;
        float pos_x = 0, pos_y = 0, pos_z = 0;
        float rot_w = 1, rot_x = 0, rot_y = 0, rot_z = 0;
        float scale_x = 1, scale_y = 1, scale_z = 1;
        float mesh_pos_x = 0, mesh_pos_y = 0, mesh_pos_z = 0;
        float mesh_rot_w = 1, mesh_rot_x = 0, mesh_rot_y = 0, mesh_rot_z = 0;
        float mesh_scale_x = 1, mesh_scale_y = 1, mesh_scale_z = 1;
        float nu = 0.0f;
        float streamwise_velocity = 0.0f;
        uint8_t streamwise_axis = 0;
        int max_particles = 0;
        bool valid = false;
    } solver_cache_;

    std::future<void> rebuild_future_;
    bool rebuild_in_progress_ = false;
    float sim_time_accumulator_ = 0.0f;
    static constexpr float target_steps_per_second_ = 900.0f;
    uint32_t next_health_check_ = 0;
};

} // namespace exd::solver::fluidx3d
