// poncelet — casing fragmentation implementation (Phase 19 item 3).
// SPDX-License-Identifier: MIT
//
// Mott mass spectrum + Gurney launch speed + a spray-direction sampler. Like
// explosion.cpp this is NOT the fp-contract-off TU: the fragment spectrum is a
// set of engineering fits (~tens of % spread) and never feeds the trajectory
// core. The seeded splitmix64 RNG keeps a spray reproducible run to run on one
// platform (PlatformStable), matching the rest of the library.
#include "poncelet/fragmentation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pon {
namespace {

constexpr Real kPi = 3.14159265358979323846;

// splitmix64 — same generator the validation fuzz sweep uses; no <random> so
// the spray is bit-reproducible.
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s(seed) {}
    std::uint64_t next() {
        s += 0x9E3779B97F4A7C15ull;
        std::uint64_t z = s;
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    Real unit() { return Real(next() >> 11) * (Real(1) / Real(9007199254740992.0)); }
    Real range(Real lo, Real hi) { return lo + (hi - lo) * unit(); }
};

// Orthonormal basis with +z' along `axis` (assumed unit).
void basis_from_axis(Vec3 axis, Vec3& e0, Vec3& e1) {
    const Vec3 up = std::fabs(axis.z) < Real(0.9) ? Vec3{0, 0, 1} : Vec3{1, 0, 0};
    e0 = normalized(cross(up, axis));
    e1 = cross(axis, e0);
}

Vec3 sphere_equiv(Real mass_kg, Real density) {
    const Real d = std::cbrt(Real(6) * mass_kg / (kPi * density));
    return {d, 0, 0};
}

} // namespace

MetersPerSec gurney_fragment_velocity(const FragmentationDesc& d,
                                      Kilograms chargeMass_kg) {
    const Real M = std::max(d.casingMass_kg, Real(0));
    const Real C = std::max(chargeMass_kg, Real(1e-9));
    const Real ratio = M / C;
    return d.gurneyVelocity_mps / std::sqrt(ratio + Real(0.5));
}

Kilograms mott_mu_kg(const FragmentationDesc& d) {
    if (d.mottMu_kg > Real(0)) return d.mottMu_kg;
    const Real t  = d.casingWallThickness_m;
    const Real di = d.casingInnerDiameter_m;
    if (t > Real(0) && di > Real(0)) {
        // µ^½ = B · t^(5/6) · d_i^(1/3) · (1 + t/d_i)   (Mott, SI-fitted B).
        const Real root = d.mottConstantB * std::pow(t, Real(5.0 / 6.0)) *
                          std::cbrt(di) * (Real(1) + t / di);
        return std::max(root * root, Real(1e-9));
    }
    // No geometry: assume the casing yields ~1200 fragments, so µ ≈ M/2400.
    return std::max(d.casingMass_kg / Real(2400), Real(1e-9));
}

std::size_t generate_fragments(const FragmentationDesc& d, Vec3 origin,
                               Vec3 sourceVelocity, Kilograms chargeMass_kg,
                               std::vector<FragmentSpec>& out) {
    out.clear();
    if (d.casingMass_kg <= Real(0)) return 0;

    const Real mu   = mott_mu_kg(d);
    const Real N0   = std::max(d.casingMass_kg / (Real(2) * mu), Real(1));
    // Fraction of the Mott population above the dust cut, and how many that is —
    // the light tail below minFragmentMass_kg is discarded (it carries little
    // mass and is not a threat), so every generated spec is a real fragment.
    const Real minM = std::max(d.minFragmentMass_kg, Real(0));
    const Real fMax = minM > Real(0) ? std::exp(-std::sqrt(minM / mu)) : Real(1);
    const Real nSig = std::max(N0 * fMax, Real(1));
    const std::uint64_t K =
        std::min<std::uint64_t>(static_cast<std::uint64_t>(nSig + Real(0.5)),
                                std::max<std::uint32_t>(d.maxFragments, 1));
    const Real represents = nSig / Real(K);

    const Real v0 = gurney_fragment_velocity(d, chargeMass_kg);
    Vec3 axis = normalized(d.sprayAxis);
    if (length_sq(axis) <= Real(0)) axis = Vec3{1, 0, 0};
    Vec3 e0, e1;
    basis_from_axis(axis, e0, e1);

    Rng rng(d.seed ^ 0x5851F42D4C957F2Dull);
    out.reserve(static_cast<std::size_t>(K));

    for (std::uint64_t i = 0; i < K; ++i) {
        // Mott quantile: K evenly-spaced points across the retained spectrum
        // (0, fMax], `f` = the fraction of fragments heavier than m_i. Each spec
        // then represents nSig/K real fragments; heaviest is i = 0.
        const Real f = fMax * (Real(i) + Real(0.5)) / Real(K);
        const Real lf = std::log(f);          // < 0
        const Real m  = mu * lf * lf;
        if (m < minM) continue;               // safety — normally unreachable now

        // Direction.
        Vec3 dir;
        const Real u1 = rng.unit(), u2 = rng.unit();
        switch (d.spray) {
            case FragmentSpray::Isotropic: {
                const Real z = Real(2) * u1 - Real(1);
                const Real r = std::sqrt(std::max(Real(0), Real(1) - z * z));
                const Real ph = Real(2) * kPi * u2;
                dir = {r * std::cos(ph), r * std::sin(ph), z};
                break;
            }
            case FragmentSpray::Cone: {
                const Real cosMax = std::cos(std::max(d.coneHalfAngle_rad, Real(0)));
                const Real ct = Real(1) - u1 * (Real(1) - cosMax);
                const Real st = std::sqrt(std::max(Real(0), Real(1) - ct * ct));
                const Real ph = Real(2) * kPi * u2;
                dir = axis * ct + e0 * (st * std::cos(ph)) + e1 * (st * std::sin(ph));
                break;
            }
            case FragmentSpray::CylinderBeam: {
                const Real psi = Real(2) * kPi * u1;
                const Vec3 radial = e0 * std::cos(psi) + e1 * std::sin(psi);
                const Real eps = (Real(2) * u2 - Real(1)) * d.beamHalfWidth_rad;
                const Real tau = d.beamForwardTilt_rad + eps;
                dir = radial * std::cos(tau) + axis * std::sin(tau);
                break;
            }
        }
        dir = normalized(dir);

        const Real scat  = Real(1) + d.velocityScatter * (Real(2) * rng.unit() - Real(1));
        const Real speed = std::max(v0 * scat, Real(0));

        FragmentSpec fs;
        fs.position        = origin;
        fs.velocity        = sourceVelocity + dir * speed;
        fs.mass_kg         = m;
        fs.diameter_m      = sphere_equiv(m, std::max(d.fragmentDensity_kgm3, Real(1))).x;
        fs.dragCoefficient = d.fragmentDragCd;
        fs.representsCount  = represents;
        out.push_back(fs);
    }
    return out.size();
}

std::vector<FragmentClass> plan_fragment_classes(
    const std::vector<FragmentSpec>& specs, std::uint32_t maxClasses,
    std::vector<std::uint32_t>* classOf) {

    std::vector<FragmentClass> classes;
    if (classOf) classOf->assign(specs.size(), 0);
    if (specs.empty()) return classes;

    Real mLo = specs.front().mass_kg, mHi = specs.front().mass_kg;
    for (const auto& s : specs) { mLo = std::min(mLo, s.mass_kg); mHi = std::max(mHi, s.mass_kg); }
    const std::uint32_t nb = std::max<std::uint32_t>(maxClasses, 1);
    const Real logLo = std::log(std::max(mLo, Real(1e-12)));
    const Real logHi = std::log(std::max(mHi, mLo * Real(1.0001) + Real(1e-12)));

    // Accumulate per-bucket mean mass so the class representative is not just the
    // bin centre.
    std::vector<Real>          sumMass(nb, 0), sumDia(nb, 0), sumCd(nb, 0);
    std::vector<std::uint32_t> cnt(nb, 0);
    std::vector<std::uint32_t> binOf(specs.size(), 0);
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const Real u = (std::log(std::max(specs[i].mass_kg, Real(1e-12))) - logLo) /
                       (logHi - logLo);
        std::uint32_t b = static_cast<std::uint32_t>(
            std::clamp(u, Real(0), Real(0.999999)) * nb);
        if (b >= nb) b = nb - 1;
        binOf[i] = b;
        sumMass[b] += specs[i].mass_kg;
        sumDia[b]  += specs[i].diameter_m;
        sumCd[b]   += specs[i].dragCoefficient;
        ++cnt[b];
    }

    std::vector<std::uint32_t> remap(nb, 0);
    for (std::uint32_t b = 0; b < nb; ++b) {
        if (!cnt[b]) continue;
        remap[b] = static_cast<std::uint32_t>(classes.size());
        FragmentClass c;
        c.mass_kg         = sumMass[b] / cnt[b];
        c.diameter_m      = sumDia[b] / cnt[b];
        c.dragCoefficient = sumCd[b] / cnt[b];
        classes.push_back(c);
    }
    if (classOf)
        for (std::size_t i = 0; i < specs.size(); ++i)
            (*classOf)[i] = remap[binOf[i]];
    return classes;
}

ProjectileType fragment_class_type(const FragmentClass& c, const std::string& id) {
    ProjectileType t;
    t.id             = id;
    t.klass          = ProjectileClass::Pellet;
    t.dragModel      = DragModel::ConstantCd;
    t.dragCoefficient = c.dragCoefficient > Real(0) ? c.dragCoefficient : Real(1.10);
    t.mass_kg        = std::max(c.mass_kg, Real(1e-6));
    t.refDiameter_m  = std::max(c.diameter_m, Real(1e-4));
    t.terminal.noseShapeFactor = 1.1;   // blunt irregular chunk
    t.terminal.hardness        = 3.0;   // hardened steel splinter
    t.terminal.deformable      = false;
    t.terminal.fragile         = false;
    t.maxLifetime_s  = 12.0;
    t.maxRange_m     = 4000.0;
    return t;
}

std::size_t spawn_fragments(Sim& sim, const std::vector<FragmentSpec>& specs,
                            FidelityTier tier, const std::string& idPrefix,
                            std::uint32_t massClasses,
                            std::vector<StateId>* outIds) {
    if (specs.empty()) return 0;

    std::vector<std::uint32_t> classOf;
    const std::vector<FragmentClass> classes =
        plan_fragment_classes(specs, massClasses, &classOf);

    std::vector<TypeId> typeIds(classes.size(), kInvalidType);
    for (std::size_t k = 0; k < classes.size(); ++k) {
        char suffix[8];
        std::snprintf(suffix, sizeof suffix, "_c%02zu", k);
        typeIds[k] = sim.registerType(fragment_class_type(classes[k], idPrefix + suffix));
    }

    std::size_t spawned = 0;
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const TypeId ty = typeIds[classOf[i]];
        if (ty == kInvalidType) continue;
        const Real speed = length(specs[i].velocity);
        if (speed <= Real(0)) continue;
        LaunchParams lp;
        lp.position  = specs[i].position;
        lp.direction = specs[i].velocity;   // registerType-normalized in spawn()
        lp.speed     = speed;
        lp.tier      = tier;
        const StateId id = sim.spawn(ty, lp);
        if (id == kInvalidState) continue;
        if (outIds) outIds->push_back(id);
        ++spawned;
    }
    return spawned;
}

} // namespace pon
