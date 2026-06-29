#pragma once

#include <exd/solver/fluidx3d/fluidx3d_system.hpp>
#include <exd/physics/solver/plugin_interface.hpp>

namespace exd::solver::fluidx3d {

/// @brief FluidX3D solver plugin implementing exd::physics::ISolverPlugin.
///
/// Wraps the FluidX3D-CLI LBM library as an Extropian solver plugin.
/// This class does NOT depend on ECS — it's pure solver interface.
class FluidX3DPlugin : public exd::physics::ISolverPlugin {
public:
    FluidX3DPlugin();
    ~FluidX3DPlugin() override;

    [[nodiscard]] std::string_view name()    const override { return "FluidX3D"; }
    [[nodiscard]] std::string_view version() const override { return "1.0"; }
    [[nodiscard]] exd::physics::PhysicsDomain domain() const override {
        return exd::physics::PhysicsDomain::FluidFlow;
    }
    [[nodiscard]] std::vector<std::string_view> supported_bc_types() const override;

    void initialize(
        const exd::physics::SolverMesh& mesh,
        std::span<const exd::physics::BoundaryCondition> bcs,
        std::span<const exd::physics::MaterialAssignment> materials,
        const exd::physics::ConfigNode& solver_params
    ) override;

    bool step(double dt) override;
    void finalize() override;

    [[nodiscard]] std::unique_ptr<exd::physics::FieldAccessor> get_field(
        const std::string& field_name) override;

    [[nodiscard]] std::unique_ptr<exd::physics::CouplingSurface> get_coupling_surface(
        const std::string& surface_name) override;

private:
    class LBM* lbm_ = nullptr;
};

} // namespace exd::solver::fluidx3d
