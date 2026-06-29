# extropian-solver-fluidx3d

**FluidX3D LBM solver plugin implementing `ext::physics::ISolverPlugin`.**

Wraps the [FluidX3D](https://github.com/ProjectPhysX/FluidX3D) GPU-accelerated Lattice Boltzmann CFD solver as an Extropian solver plugin.

## Usage

```cpp
#include <ext/solver/fluidx3d/fluidx3d_plugin.hpp>
#include <ext/physics/solver/solver_manager.hpp>

auto& manager = ext::physics::SolverManager::instance();
auto plugin = manager.create("fluidx3d");

plugin->initialize(mesh, bcs, materials, params);
while (plugin->step(dt)) { /* ... */ }
plugin->finalize();
```

Also provides a ready-to-use ECS system:

```cpp
#include <ext/solver/fluidx3d/fluidx3d_system.hpp>
graph.add<ext::solver::fluidx3d::FluidX3DSystem>().in_mode(SimMode::Simulate);
```

## Building

```bash
cmake -S . -B build -G Ninja -DFLUIDX3D_SOURCE_DIR=/path/to/FluidX3D-CLI
cmake --build build
```

Requires: `extropian-core`, `extropian-physics`, OpenCL, FluidX3D-CLI.

## License

Business Source License 1.1 — see [LICENSE](LICENSE).
Converts to Apache 2.0 on 2029-05-26.
