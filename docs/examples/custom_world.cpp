// poncelet example — implement pon::World against your own geometry.
// SPDX-License-Identifier: MIT
//
// A ground plane at y=0 and a vertical wall at x=25. The projectile is an
// arrow; we catch the Stopped event and report where it landed.
#include <poncelet/poncelet.hpp>

#include <array>
#include <cmath>
#include <cstdio>

namespace {

// The simplest useful World: a ground plane and one wall. Real callers forward
// to their scene's broad-phase + segment intersection.
class RangeWorld final : public pon::World {
public:
    RangeWorld() {
        ground_.name = "turf";
        ground_.behaviour = pon::MaterialBehaviour::Granular;
        ground_.density_kgm3 = 1500;
        ground_.strength_Pa = 2.0e6;
        ground_.thickness_m = -1.0; // bulk earth — never perforates

        wall_.name = "oak_plank";
        wall_.behaviour = pon::MaterialBehaviour::Fibrous;
        wall_.density_kgm3 = 750;
        wall_.strength_Pa = 9.0e7;
        wall_.thickness_m = 0.05;
    }

    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        double bestT = 1.0;
        bool hit = false;

        // Ground plane y = 0, only when crossing downward.
        if (a.y > 0.0 && b.y <= 0.0) {
            const double t = a.y / (a.y - b.y);
            if (t < bestT) {
                bestT = t; hit = true;
                out.point = a + (b - a) * t;
                out.normal = {0, 1, 0};
                out.surface = kGround;
            }
        }
        // Wall x = 25, spanning 0..5 m high.
        if ((a.x - 25.0) * (b.x - 25.0) < 0.0) {
            const double t = (25.0 - a.x) / (b.x - a.x);
            const pon::Vec3 p = a + (b - a) * t;
            if (t < bestT && p.y >= 0.0 && p.y <= 5.0) {
                bestT = t; hit = true;
                out.point = p;
                out.normal = {a.x < 25.0 ? -1.0 : 1.0, 0, 0};
                out.surface = kWall;
            }
        }

        if (hit) {
            out.t = bestT;
            out.surfaceVelocity = {0, 0, 0}; // static geometry
        }
        return hit;
    }

    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }

    const pon::Material& material(pon::SurfaceId s) const override {
        return s == kWall ? wall_ : ground_;
    }

private:
    static constexpr pon::SurfaceId kGround = 0;
    static constexpr pon::SurfaceId kWall   = 1;
    pon::Material ground_;
    pon::Material wall_;
};

} // namespace

int main() {
    pon::Sim sim;

    pon::ProjectileType arrow;
    arrow.id    = "longbow_bodkin_60g";
    arrow.klass = pon::ProjectileClass::Arrow; // mass/diameter/Cd/speed defaulted
    const pon::TypeId kArrow = sim.registerType(arrow);

    // Loose it at ~10 degrees of elevation.
    const double elev = 10.0 * 3.14159265 / 180.0;
    pon::LaunchParams lp;
    lp.position  = {0, 1.5, 0};
    lp.direction = {std::cos(elev), std::sin(elev), 0};
    const pon::StateId a = sim.spawn(kArrow, lp);

    RangeWorld world;
    pon::VectorEventSink sink;

    for (int i = 0; i < 2000 && sim.state(a).alive; ++i)
        sim.step(1.0 / 240.0, world, sink);

    for (const pon::Event& e : sink.events) {
        const char* what = "event";
        switch (e.type) {
            case pon::EventType::Stopped:        what = "stopped";     break;
            case pon::EventType::Embedded:       what = "embedded";    break;
            case pon::EventType::Ricochet:       what = "ricocheted";  break;
            case pon::EventType::Perforated:     what = "perforated";  break;
            case pon::EventType::SurfaceCrossed: what = "crossed";     break;
            case pon::EventType::Expired:        what = "expired";     break;
            default: break;
        }
        std::printf("%-11s at x=%.2f y=%.2f  after %.2f s"
                    "   (E=%.0f J, v_res=%.0f m/s, depth=%.3f m)\n",
                    what, e.point.x, e.point.y, e.time_s,
                    e.energy_J, e.residualSpeed_mps, e.channelDepth_m);
    }
    return 0;
}
