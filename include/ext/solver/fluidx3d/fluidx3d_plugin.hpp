#pragma once

#include <ext/solver/fluidx3d/fluidx3d_system.hpp>
#include <ext/physics/solver/plugin_interface.hpp>

namespace ext::solver::fluidx3d {

/// @brief FluidX3D solver plugin implementing ext::physics::ISolverPlugin.
///
/// Wraps the FluidX3D-CLI LBM library as an Extropian solver plugin.
/// This class does NOT depend on ECS — it's pure solver interface.
class FluidX3DPlugin : public ext::physics::ISolverPlugin {
public:
    FluidX3DPlugin();
    ~FluidX3DPlugin() override;

    [[nodiscard]] std::string_view name()    const override { return "FluidX3D"; }
    [[nodiscard]] std::string_view version() const override { return "1.0"; }
    [[nodiscard]] ext::physics::PhysicsDomain domain() const override {
        return ext::physics::PhysicsDomain::FluidFlow;
    }
    [[nodiscard]] std::vector<std::string_view> supported_bc_types() const override;

    void initialize(
        const ext::physics::SolverMesh& mesh,
        std::span<const ext::physics::BoundaryCondition> bcs,
        std::span<const ext::physics::MaterialAssignment> materials,
        const ext::physics::ConfigNode& solver_params
    ) override;

    bool step(double dt) override;
    void finalize() override;

    [[nodiscard]] std::unique_ptr<ext::physics::FieldAccessor> get_field(
        const std::string& field_name) override;

    [[nodiscard]] std::unique_ptr<ext::physics::CouplingSurface> get_coupling_surface(
        const std::string& surface_name) override;

private:
    class LBM* lbm_ = nullptr;
};

} // namespace ext::solver::fluidx3d
