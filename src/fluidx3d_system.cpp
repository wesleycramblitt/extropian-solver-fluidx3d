#include <exd/solver/fluidx3d/fluidx3d_system.hpp>
#include <exd/ecs/registry.hpp>
#include <cstdio>

namespace exd::solver::fluidx3d {

FluidX3DSystem::FluidX3DSystem(exd::render::GraphicsContext& ctx)
    : ctx_(ctx) {}

FluidX3DSystem::~FluidX3DSystem() {
    destroy_solver();
}

void FluidX3DSystem::update(exd::ecs::Registry& registry, float dt) {
    // Iterate all simulation entities
    for (auto [entity] : registry.view<FluidX3DSolverConfig, FluidPhysics, SimulationInfo>().each()) {
        auto& info = registry.get<SimulationInfo>(entity);
        auto& physics = registry.get<FluidPhysics>(entity);

        // Find domain box entity (has SimulationDomain + Transform + SimulationReference)
        for (auto [child] : registry.view<SimulationDomain, Transform, SimulationReference>().each()) {
            auto& ref = registry.get<SimulationReference>(child);
            if (ref.simulation_entity_id != entity.id) continue;

            auto& domain = registry.get<SimulationDomain>(child);
            auto& xform = registry.get<Transform>(child);

            // First-time: create solver
            if (!lbm_) {
                create_solver(registry, domain, physics, info, xform);
            }

            // Check async rebuilds
            check_async_rebuild(info);

            // TODO: Detect config changes → rebuild
            // TODO: Step solver, read back particles, update volume field
        }
    }
}

void FluidX3DSystem::create_solver(exd::ecs::Registry& registry,
                                    const SimulationDomain& domain,
                                    const FluidPhysics& physics,
                                    SimulationInfo& info,
                                    const Transform& xform) {
    // TODO: Port the 841-line createSolver() from the original code.
    // This calls into FluidX3D-CLI shared library to build an LBM.
    std::printf("[FluidX3DSystem] create_solver: %dx%dx%d, nu=%.4f\n",
                domain.nx, domain.ny, domain.nz, physics.nu);
    info.status = SimulationStatus::Stopped;
}

void FluidX3DSystem::destroy_solver() {
    // TODO: Delete LBM, clear caches.
    solver_cache_.valid = false;
}

void FluidX3DSystem::launch_async_rebuild(exd::ecs::Registry& registry,
                                           const SimulationDomain& domain,
                                           const FluidPhysics& physics,
                                           SimulationInfo& info,
                                           const Transform& xform) {
    // TODO: Port the async rebuild logic.
}

void FluidX3DSystem::check_async_rebuild(SimulationInfo& info) {
    if (!rebuild_in_progress_) return;
    if (rebuild_future_.valid() &&
        rebuild_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        rebuild_future_.get();
        rebuild_in_progress_ = false;
        info.status = SimulationStatus::Stopped;
    }
}

} // namespace exd::solver::fluidx3d
