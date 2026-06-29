#include <ext/solver/fluidx3d/fluidx3d_plugin.hpp>
#include <cstdio>

namespace ext::solver::fluidx3d {

FluidX3DPlugin::FluidX3DPlugin() {
    std::printf("[FluidX3DPlugin] Created.\n");
}

FluidX3DPlugin::~FluidX3DPlugin() {
    finalize();
    std::printf("[FluidX3DPlugin] Destroyed.\n");
}

std::vector<std::string_view> FluidX3DPlugin::supported_bc_types() const {
    return {"fixed_value", "zero_gradient", "slip", "inlet_velocity", "outlet_pressure"};
}

void FluidX3DPlugin::initialize(
    const ext::physics::SolverMesh& mesh,
    std::span<const ext::physics::BoundaryCondition> bcs,
    std::span<const ext::physics::MaterialAssignment> materials,
    const ext::physics::ConfigNode& solver_params)
{
    // TODO: Build LBM solver from mesh + BCs + params.
    // This is where the 841-line FluidX3DSystem::createSolver code goes,
    // adapted to use ISolverPlugin interfaces instead of direct ECS access.
    std::printf("[FluidX3DPlugin] initialize() — mesh nodes=%zu, BCs=%zu, params=%zu\n",
                mesh.node_coords.size(),
                bcs.size(),
                solver_params.size());
}

bool FluidX3DPlugin::step(double dt) {
    // TODO: Advance LBM one timestep.
    return false; // false = converged/stopped
}

void FluidX3DPlugin::finalize() {
    // TODO: Delete LBM instance.
}

std::unique_ptr<ext::physics::FieldAccessor>
FluidX3DPlugin::get_field(const std::string& field_name) {
    // TODO: Return velocity/pressure/density fields from LBM.
    return nullptr;
}

std::unique_ptr<ext::physics::CouplingSurface>
FluidX3DPlugin::get_coupling_surface(const std::string& surface_name) {
    // TODO: Return coupling surface mesh + fields.
    return nullptr;
}

} // namespace ext::solver::fluidx3d
