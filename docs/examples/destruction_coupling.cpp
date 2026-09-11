// poncelet example — destruction coupling: the kinetic impact impulse and the
// explosive blast load both drive a toy structural-failure model (Phase 19
// item 6, §8a).
//
// A wall of stacked blocks, each held by a bond that fails past an impulse
// budget. We (1) fire a rifle round through it — the Perforated event's
// impulse_Ns knocks its block loose — and (2) detonate a grenade in front of
// it and let pon::Burst::loadOnBody topple the nearest blocks. This is exactly
// the seam a game engine wires into its chunk / fracture system.
//
// Build (standalone): configured by default. Run:
//   ./build/<cfg>/poncelet_example_destruction_coupling
#include <poncelet/poncelet.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

// A 5-wide × 4-high wall centred on x = 20, in the y-z plane; each block is
// 0.4 m, bond fails at 3.0 N·s of accumulated impulse.
struct Block {
    pon::Vec3 centre;
    double    impulseBudget_Ns = 7.0;
    double    accumulated_Ns   = 0.0;
    bool      detached         = false;
};

struct Wall final : pon::World {
    std::vector<Block> blocks;
    pon::Material brick = [] {
        pon::Material m;
        m.name = "brick"; m.behaviour = pon::MaterialBehaviour::Brittle;
        m.density_kgm3 = 1900; m.strength_Pa = 1.4e7; m.thickness_m = 0.22;
        return m;
    }();
    Wall() {
        for (int j = 0; j < 4; ++j)
            for (int i = 0; i < 5; ++i)
                blocks.push_back({{20.0, 0.2 + j * 0.4, (i - 2) * 0.4}});
    }
    bool raycast(pon::Vec3 a, pon::Vec3 b, pon::HitResult& out) const override {
        if ((a.x - 20.0) * (b.x - 20.0) >= 0.0) return false;
        const double f = (20.0 - a.x) / (b.x - a.x);
        out.point   = a + (b - a) * f;
        out.normal  = {a.x < 20.0 ? -1.0 : 1.0, 0, 0};
        out.t       = f;
        out.surface = 1;
        return true;
    }
    pon::MediumId mediumAt(pon::Vec3) const override { return pon::kMediumAir; }
    const pon::Material& material(pon::SurfaceId) const override { return brick; }

    // Deposit an impulse at a point; the nearest standing block takes it and
    // detaches once its budget is spent.
    void deposit(pon::Vec3 at, double mag_Ns) {
        Block* best = nullptr;
        double bestD2 = 1e9;
        for (Block& bl : blocks) {
            if (bl.detached) continue;
            const pon::Vec3 d = bl.centre - at;
            const double d2 = pon::dot(d, d);
            if (d2 < bestD2) { bestD2 = d2; best = &bl; }
        }
        if (!best) return;
        best->accumulated_Ns += mag_Ns;
        if (best->accumulated_Ns >= best->impulseBudget_Ns) best->detached = true;
    }
    int detachedCount() const {
        int n = 0;
        for (const Block& bl : blocks) n += bl.detached;
        return n;
    }
};

} // namespace

int main() {
    Wall world;

    // --- Channel 1: a kinetic round punches through -------------------------
    pon::Sim sim;
    pon::ProjectileType rifle;
    rifle.id = "762x51"; rifle.klass = pon::ProjectileClass::Bullet;
    rifle.dragModel = pon::DragModel::G7; rifle.ballisticCoefficient = 0.24;
    rifle.mass_kg = 0.0113; rifle.refDiameter_m = 0.00782;
    const pon::TypeId rid = sim.registerType(rifle);

    pon::LaunchParams lp;
    lp.position = {0, 1.0, 0.0}; lp.direction = {1, 0, 0}; lp.speed = 833.0;
    lp.tier = pon::FidelityTier::Integrated;
    const pon::StateId h = sim.spawn(rid, lp);

    pon::VectorEventSink ev;
    for (int i = 0; i < 8000 && sim.state(h).alive; ++i)
        sim.step(1.0 / 2000.0, world, ev);

    double kineticImpulse = 0.0;
    for (const pon::Event& e : ev.events) {
        if (e.type == pon::EventType::Perforated ||
            e.type == pon::EventType::Embedded ||
            e.type == pon::EventType::Stopped) {
            kineticImpulse = pon::length(e.impulse_Ns);
            world.deposit(e.point, kineticImpulse);
        }
    }
    const int afterRound = world.detachedCount();
    std::printf("rifle round: channel impulse %.2f N.s -> %d block(s) knocked loose\n",
                kineticImpulse, afterRound);

    // --- Channel 2: a grenade goes off 3 m in front of the wall ------------
    pon::WarheadDesc nade;
    nade.chargeMass_kg  = 0.060;   // ~60 g Comp B
    nade.tntEquivalence = 1.15;
    const pon::Vec3 burstAt{18.5, 0.6, -0.6};  // 1.5 m out, off to one corner
    pon::Burst burst(burstAt, nade, sim.environment(), 0.0);

    int blastFailures = 0;
    for (Block& bl : world.blocks) {
        if (bl.detached) continue;
        pon::BlastTarget t;
        t.centroid = bl.centre;
        t.area_m2  = 0.16;         // 0.4 m face
        t.mass_kg  = 12.0;
        const double los = pon::blast_line_of_sight(world, burstAt, bl.centre);
        const pon::BlastLoad load = burst.loadOnBody(t, los);
        const double j = pon::length(load.impulse_Ns);
        bl.accumulated_Ns += j;
        if (bl.accumulated_Ns >= bl.impulseBudget_Ns && !bl.detached) {
            bl.detached = true;
            ++blastFailures;
        }
    }
    std::printf("grenade blast: %d more block(s) failed (%d / %zu wall gone)\n",
                blastFailures, world.detachedCount(), world.blocks.size());

    // ctest gate: the round drops exactly its block, and the blast — much more
    // energetic, spread over the wall — brings down several more without
    // levelling the whole thing.
    const int total = static_cast<int>(world.blocks.size());
    const bool ok = kineticImpulse > 0.0 && kineticImpulse < 0.0113 * 833.0 * 1.02 &&
                    afterRound == 1 &&
                    blastFailures >= 3 && world.detachedCount() < total;
    return ok ? 0 : 1;
}
