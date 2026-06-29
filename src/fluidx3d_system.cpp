#include <exd/solver/fluidx3d/fluidx3d_system.hpp>
#include <exd/solver/fluidx3d/components.hpp>
#include <exd/render/graphics_context.hpp>
#include <exd/render/mesh.hpp>
#include <exd/render/vertex.hpp>
#include <exd/render/components.hpp>

#define Mesh F3D_Mesh
#include "lbm.hpp"
#include "defines.hpp"
#undef Mesh

#include <exd/math/quat.hpp>
#include <exd/math/vec3.hpp>

#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <vector>
#include <array>
#include <future>

namespace exd::solver::fluidx3d {

FluidX3DSystem::FluidX3DSystem(exd::render::GraphicsContext& ctx) : ctx_(ctx) {}
FluidX3DSystem::~FluidX3DSystem() { destroy_solver(); }

void FluidX3DSystem::destroy_solver() {
    if (lbm_) { delete lbm_; lbm_ = nullptr; }
    prev_particle_x_.clear();
    solver_cache_.valid = false;
}

void FluidX3DSystem::launch_async_rebuild(exd::ecs::Registry& registry,
                                           const SimulationDomain& domain,
                                           const FluidPhysics& phys,
                                           SimulationInfo& info,
                                           const Transform& xform) {
    if (rebuild_in_progress_) return;

    const uint nx = static_cast<uint>(domain.nx), ny = static_cast<uint>(domain.ny), nz = static_cast<uint>(domain.nz);
    const float nu = phys.nu, target_u = phys.streamwise_velocity;
    const uint8_t axis = phys.streamwise_axis;

    std::string stl_path;
    float mesh_wx = 0, mesh_wy = 0, mesh_wz = 0, mesh_sx = 1, mesh_sy = 1, mesh_sz = 1;
    math::Quat mesh_rot{1, 0, 0, 0};
    for (auto fe : registry.view<MeshAsset, Transform>()) {
        stl_path = registry.get<MeshAsset>(fe).path;
        auto& mt = registry.get<Transform>(fe);
        mesh_wx = mt.pos_x; mesh_wy = mt.pos_y; mesh_wz = mt.pos_z;
        mesh_sx = mt.scale_x; mesh_sy = mt.scale_y; mesh_sz = mt.scale_z;
        mesh_rot = {mt.rot_w, mt.rot_x, mt.rot_y, mt.rot_z};
        break;
    }

    uint particles_N = 10000u;
    for (auto e : registry.view<ParticleCloud>())
        { particles_N = static_cast<uint>(registry.get<ParticleCloud>(e).max_particles); break; }

    const float dp_x = xform.pos_x, dp_y = xform.pos_y, dp_z = xform.pos_z;
    const float ds_x = xform.scale_x, ds_y = xform.scale_y, ds_z = xform.scale_z;
    const math::Quat dr{xform.rot_w, xform.rot_x, xform.rot_y, xform.rot_z};

    const float duct_w = static_cast<float>(ny - 2u), duct_h = static_cast<float>(nz - 2u);
    const float force_mag = ext::core::units::f_from_u_rectangular_duct(duct_w, duct_h, 1.0f, nu, target_u);
    float fx = 0, fy = 0, fz = 0;
    switch (axis) { case 0: fx = -force_mag; break; case 1: fy = -force_mag; break; case 2: fz = -force_mag; break; default: fx = -force_mag; break; }

    if (info.status != SimulationStatus::Stopped) {
        info.status = SimulationStatus::Stopped;
        if (lbm_) lbm_->reset();
    }
    destroy_solver();

    solver_cache_ = {static_cast<int>(nx), static_cast<int>(ny), static_cast<int>(nz),
                     dp_x, dp_y, dp_z, dr.w, dr.x, dr.y, dr.z,
                     ds_x, ds_y, ds_z,
                     mesh_wx, mesh_wy, mesh_wz,
                     mesh_rot.w, mesh_rot.x, mesh_rot.y, mesh_rot.z,
                     mesh_sx, mesh_sy, mesh_sz,
                     nu, target_u, axis, static_cast<int>(particles_N), true};

    rebuild_in_progress_ = true;

    rebuild_future_ = std::async(std::launch::async, [this,
            nx, ny, nz, nu, fx, fy, fz, particles_N,
            stl_path, mesh_wx, mesh_wy, mesh_wz, mesh_sx, mesh_rot,
            dp_x, dp_y, dp_z, ds_x, ds_y, ds_z, dr, force_mag, target_u]() {
        LBM* new_lbm = new LBM(nx, ny, nz, nu, fx, fy, fz, particles_N);
        const ulong N = new_lbm->get_N();
        for (uint z = 0; z < nz; z++)
            for (uint y = 0; y < ny; y++)
                for (uint x = 0; x < nx; x++)
                    if (y == 0 || y == ny-1 || z == 0 || z == nz-1) {
                        ulong i = (ulong)x + ((ulong)y + (ulong)z * ny) * nx;
                        new_lbm->flags[i] = TYPE_S;
                    }
        if (!stl_path.empty()) {
            float stl_min[3]={1e30f,1e30f,1e30f}, stl_max[3]={-1e30f,-1e30f,-1e30f};
            {
                std::ifstream file(stl_path, std::ios::binary);
                if (file) {
                    file.seekg(0, std::ios::end); size_t sz = file.tellg(); file.seekg(0, std::ios::beg);
                    std::vector<char> buf(sz); file.read(buf.data(), sz);
                    if (sz >= 84) {
                        uint32_t ntri = *(uint32_t*)(buf.data()+80);
                        const char* p = buf.data()+84;
                        for (uint32_t i = 0; i < ntri && i < 100000; i++, p+=50) {
                            const float* f = (const float*)p;
                            for (int v=0; v<3; v++) for (int c=0; c<3; c++) {
                                float val = f[3+v*3+c];
                                if (val < stl_min[c]) stl_min[c] = val;
                                if (val > stl_max[c]) stl_max[c] = val;
                            }
                        }
                    }
                }
            }
            float native_size = std::max(std::max(stl_max[0]-stl_min[0], stl_max[1]-stl_min[1]), stl_max[2]-stl_min[2]);
            math::Vec3 rel{mesh_wx-dp_x, mesh_wy-dp_y, mesh_wz-dp_z};
            math::Quat inv_dr{dr.w, -dr.x, -dr.y, -dr.z};
            math::Vec3 rotated = inv_dr * rel;
            math::Vec3 grid_center{rotated.x/ds_x, rotated.y/ds_y, rotated.z/ds_z};
            grid_center.x += (float)nx*0.5f; grid_center.y += (float)ny*0.5f; grid_center.z += (float)nz*0.5f;

            math::Quat grid_rot = inv_dr * mesh_rot;
            math::Vec3 rx = grid_rot.right(), ry = grid_rot.up(), rz = grid_rot * math::Vec3{0,0,1};
            float3x3 mrot(rx.x, ry.x, rz.x, rx.y, ry.y, rz.y, rx.z, ry.z, rz.z);

            float grid_scale = native_size * mesh_sx / ds_x;
            float stl_to_grid = grid_scale / native_size;
            float stl_cx=(stl_min[0]+stl_max[0])*0.5f, stl_cy=(stl_min[1]+stl_max[1])*0.5f, stl_cz=(stl_min[2]+stl_max[2])*0.5f;
            math::Vec3 stl_gc{stl_cx, stl_cy, stl_cz};
            math::Vec3 rot_gc = grid_rot * stl_gc;
            float3 final_center(grid_center.x+rot_gc.x*stl_to_grid, grid_center.y+rot_gc.y*stl_to_grid, grid_center.z+rot_gc.z*stl_to_grid);
            new_lbm->voxelize_stl(stl_path, final_center, mrot, grid_scale, TYPE_S);
        }
        lbm_ = new_lbm;
    });
}

void FluidX3DSystem::check_async_rebuild(SimulationInfo& info) {
    if (!rebuild_in_progress_) return;
    if (rebuild_future_.valid() && rebuild_future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        rebuild_future_.get();
        prev_particle_x_.clear();
        info.status = SimulationStatus::Stopped;
        info.current_step = 0;
        next_health_check_ = 0;
        sim_time_accumulator_ = 0.0f;
        rebuild_in_progress_ = false;
        std::printf("[LBM] Async rebuild swapped in.\n");
    }
}

void FluidX3DSystem::create_solver(exd::ecs::Registry& registry,
                                    const SimulationDomain& domain,
                                    const FluidPhysics& phys,
                                    SimulationInfo& info,
                                    const Transform& xform) {
    const uint nx = static_cast<uint>(domain.nx), ny = static_cast<uint>(domain.ny), nz = static_cast<uint>(domain.nz);
    const float nu = phys.nu, target_u = phys.streamwise_velocity;
    const float duct_w = static_cast<float>(ny - 2u), duct_h = static_cast<float>(nz - 2u);
    const float force_mag = ext::core::units::f_from_u_rectangular_duct(duct_w, duct_h, 1.0f, nu, target_u);

    float fx = 0, fy = 0, fz = 0;
    switch (phys.streamwise_axis) { case 0: fx=-force_mag; break; case 1: fy=-force_mag; break; case 2: fz=-force_mag; break; default: fx=-force_mag; break; }

    uint particles_N = 100000u;
    for (auto e : registry.view<ParticleCloud>())
        { particles_N = static_cast<uint>(registry.get<ParticleCloud>(e).max_particles); break; }

    std::printf("[LBM] nu=%.4f target_u=%.4f duct=%ux%u fx=%.8f particles=%u\n",
                nu, target_u, static_cast<uint>(duct_w), static_cast<uint>(duct_h), force_mag, particles_N);

    lbm_ = new LBM(nx, ny, nz, nu, fx, fy, fz, particles_N);

    const ulong N = lbm_->get_N();
    for (uint z = 0; z < nz; z++)
        for (uint y = 0; y < ny; y++)
            for (uint x = 0; x < nx; x++)
                if (y == 0 || y == ny-1 || z == 0 || z == nz-1) {
                    ulong i = (ulong)x + ((ulong)y + (ulong)z * ny) * nx;
                    lbm_->flags[i] = TYPE_S;
                }

    Transform* mesh_xform = nullptr;
    std::string stl_path;
    for (auto fe : registry.view<MeshAsset, Transform>()) {
        stl_path = registry.get<MeshAsset>(fe).path;
        mesh_xform = &registry.get<Transform>(fe);
        break;
    }

    if (mesh_xform && !stl_path.empty()) {
        float stl_min[3]={1e30f,1e30f,1e30f}, stl_max[3]={-1e30f,-1e30f,-1e30f};
        {
            std::ifstream file(stl_path, std::ios::binary);
            if (file) {
                file.seekg(0, std::ios::end); size_t sz = file.tellg(); file.seekg(0, std::ios::beg);
                std::vector<char> buf(sz); file.read(buf.data(), sz);
                if (sz >= 84) {
                    uint32_t ntri = *(uint32_t*)(buf.data()+80);
                    const char* p = buf.data()+84;
                    for (uint32_t i=0; i<ntri && i<100000; i++, p+=50) {
                        const float* f = (const float*)p;
                        for (int v=0; v<3; v++) for (int c=0; c<3; c++) {
                            float val = f[3+v*3+c];
                            if (val<stl_min[c]) stl_min[c]=val;
                            if (val>stl_max[c]) stl_max[c]=val;
                        }
                    }
                }
            }
        }
        float native_size = std::max(std::max(stl_max[0]-stl_min[0], stl_max[1]-stl_min[1]), stl_max[2]-stl_min[2]);

        const float dp_x=xform.pos_x, dp_y=xform.pos_y, dp_z=xform.pos_z;
        const float ds_x=xform.scale_x, ds_y=xform.scale_y, ds_z=xform.scale_z;
        const math::Quat dr{xform.rot_w, xform.rot_x, xform.rot_y, xform.rot_z};

        math::Vec3 rel{mesh_xform->pos_x-dp_x, mesh_xform->pos_y-dp_y, mesh_xform->pos_z-dp_z};
        math::Quat inv_dr{dr.w, -dr.x, -dr.y, -dr.z};
        math::Vec3 rotated = inv_dr * rel;
        math::Vec3 grid_center{rotated.x/ds_x, rotated.y/ds_y, rotated.z/ds_z};
        grid_center.x += (float)nx*0.5f; grid_center.y += (float)ny*0.5f; grid_center.z += (float)nz*0.5f;

        math::Quat grid_rot = inv_dr * math::Quat{mesh_xform->rot_w, mesh_xform->rot_x, mesh_xform->rot_y, mesh_xform->rot_z};
        math::Vec3 rx = grid_rot.right(), ry = grid_rot.up(), rz = grid_rot * math::Vec3{0,0,1};
        float3x3 mrot(rx.x, ry.x, rz.x, rx.y, ry.y, rz.y, rx.z, ry.z, rz.z);

        float grid_scale = native_size * mesh_xform->scale_x / ds_x;
        float stl_to_grid = grid_scale / native_size;
        float stl_cx=(stl_min[0]+stl_max[0])*0.5f, stl_cy=(stl_min[1]+stl_max[1])*0.5f, stl_cz=(stl_min[2]+stl_max[2])*0.5f;
        math::Vec3 stl_gc{stl_cx, stl_cy, stl_cz};
        math::Vec3 rot_gc = grid_rot * stl_gc;
        float3 final_center(grid_center.x+rot_gc.x*stl_to_grid, grid_center.y+rot_gc.y*stl_to_grid, grid_center.z+rot_gc.z*stl_to_grid);

        lbm_->voxelize_stl(stl_path, final_center, mrot, grid_scale, TYPE_S);
    }

    int solid_count = 0;
    for (ulong i = 0; i < N; i++) if (lbm_->flags[i] == TYPE_S) solid_count++;
    std::printf("[LBM] Total TYPE_S cells: %d / %llu\n", solid_count, (unsigned long long)N);
    std::printf("[LBM] Ready.\n");

    solver_cache_ = {static_cast<int>(nx), static_cast<int>(ny), static_cast<int>(nz),
                     xform.pos_x, xform.pos_y, xform.pos_z,
                     xform.rot_w, xform.rot_x, xform.rot_y, xform.rot_z,
                     xform.scale_x, xform.scale_y, xform.scale_z,
                     mesh_xform ? mesh_xform->pos_x : 0.0f,
                     mesh_xform ? mesh_xform->pos_y : 0.0f,
                     mesh_xform ? mesh_xform->pos_z : 0.0f,
                     mesh_xform ? mesh_xform->rot_w : 1.0f,
                     mesh_xform ? mesh_xform->rot_x : 0.0f,
                     mesh_xform ? mesh_xform->rot_y : 0.0f,
                     mesh_xform ? mesh_xform->rot_z : 0.0f,
                     mesh_xform ? mesh_xform->scale_x : 1.0f,
                     mesh_xform ? mesh_xform->scale_y : 1.0f,
                     mesh_xform ? mesh_xform->scale_z : 1.0f,
                     phys.nu, phys.streamwise_velocity, phys.streamwise_axis,
                     static_cast<int>(particles_N), true};
    info.status = SimulationStatus::Stopped;
    info.current_step = 0;
}

// ── Per-frame update (abbreviated — core step loop, particle readback, volume upload) ──
void FluidX3DSystem::update(exd::ecs::Registry& registry, float dt) {
    for (auto simEntity : registry.view<FluidX3DSolverConfig, FluidPhysics, SimulationInfo>()) {
        if (registry.has<Disabled>(simEntity)) continue;
        auto& info = registry.get<SimulationInfo>(simEntity);

        // Find domain box entity
        exd::ecs::Entity domainEntity{};
        for (auto db : registry.view<SimulationDomain, Transform, SimulationReference>()) {
            if (registry.get<SimulationReference>(db).simulation_entity_id == simEntity.id) {
                domainEntity = db; break;
            }
        }
        if (!registry.valid(domainEntity)) continue;

        auto& domain = registry.get<SimulationDomain>(domainEntity);
        auto& xform = registry.get<Transform>(domainEntity);

        // First-time: create solver
        bool first_time = !registry.has<Renderable>(domainEntity);
        if (first_time) {
            create_solver(registry, domain, registry.get<FluidPhysics>(simEntity), info, xform);
        }

        check_async_rebuild(info);

        // Detect runtime edits that need rebuild (simplified)
        if (lbm_ && solver_cache_.valid && !rebuild_in_progress_) {
            auto& phys = registry.get<FluidPhysics>(simEntity);
            bool dims_changed = (solver_cache_.nx != domain.nx || solver_cache_.ny != domain.ny || solver_cache_.nz != domain.nz);
            bool pos_changed = (std::abs(solver_cache_.pos_x - xform.pos_x) > 1e-4f);
            bool phys_changed = (std::abs(solver_cache_.nu - phys.nu) > 1e-6f ||
                                 std::abs(solver_cache_.streamwise_velocity - phys.streamwise_velocity) > 1e-6f);
            if (dims_changed || pos_changed || phys_changed) {
                launch_async_rebuild(registry, domain, phys, info, xform);
                continue;
            }
        }

        if (rebuild_in_progress_) continue;

        // Time-based stepping
        if (lbm_ && info.status == SimulationStatus::Running) {
            sim_time_accumulator_ += dt;
            const float step_interval = 1.0f / target_steps_per_second_;
            uint32_t steps = 0;
            while (sim_time_accumulator_ >= step_interval && steps < 30) {
                steps++; sim_time_accumulator_ -= step_interval;
            }
            if (sim_time_accumulator_ > step_interval * 3.0f) sim_time_accumulator_ = step_interval * 3.0f;

            if (steps > 0) {
                lbm_->run(steps, 1000000);
                info.current_step += steps;
            }

            // Health check every ~500 steps
            if (info.current_step >= next_health_check_) {
                next_health_check_ = info.current_step + 500;
                lbm_->u.read_from_device();
                lbm_->rho.read_from_device();
                lbm_->flags.read_from_device();
                const ulong N = lbm_->get_N();
                float u_max = 0;
                size_t nonfinite_count = 0, fluid_count = 0;
                for (ulong i = 0; i < N; ++i) {
                    if (lbm_->flags[i] & TYPE_S) continue;
                    fluid_count++;
                    float speed = std::sqrt(lbm_->u.x[i]*lbm_->u.x[i] + lbm_->u.y[i]*lbm_->u.y[i] + lbm_->u.z[i]*lbm_->u.z[i]);
                    if (!std::isfinite(speed)) nonfinite_count++;
                    if (speed > u_max) u_max = speed;
                }
                if (nonfinite_count > 0 || u_max > 0.3f) {
                    info.status = SimulationStatus::Error;
                    std::fprintf(stderr, "[LBM] UNSTABLE at step=%u umax=%.4f nonfinite=%zu\n", info.current_step, u_max, nonfinite_count);
                }
            }

            // Particle readback (abbreviated — full 80-line version in original)
            // Volume texture upload  (abbreviated — full 30-line version in original)
        }
    }
}

} // namespace exd::solver::fluidx3d
