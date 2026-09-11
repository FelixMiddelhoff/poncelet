// poncelet — plain-data row structs for the baked tables (src/generated/
// data_tables.inc). Internal. Translated into the public ProjectileType /
// Material / BallProfile by src/catalog.cpp.
// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>

namespace pon::detail {

struct RawClassDefault {
    const char* klass;
    double      refDiameter_m;
    double      mass_kg;
    const char* dragModel;
    double      dragCoefficient;
    double      ballisticCoefficient;
    double      muzzleSpeed_mps;
    double      noseShapeFactor;
};

struct RawProjectile {
    const char* id;
    const char* klass;
    const char* dragModel;
    double      mass_kg;
    double      refDiameter_m;
    double      ballisticCoefficient; // <=0 => unset
    double      dragCoefficient;      // <=0 => unset
    const char* ballProfile;          // "" => none
    double      muzzleSpeed_mps;      // <=0 => unset
    double      noseShapeFactor;
    double      hardness;
    int         deformable;
    int         fragile;
    double      spinRate_radps;       // <=0 => unset (derive from twist)
    const char* spinAxisMode;
    double      twistRate_m;
};

struct RawCurvePt { double x, y; };

struct RawBallProfile {
    const char*       id;
    const char*       shape;
    double            diameter_m;
    double            mass_kg;
    double            typicalSpeed_mps;
    const char*       abscissa;
    const RawCurvePt* cdCurve;
    std::size_t       cdCount;
    const RawCurvePt* clCurve;
    std::size_t       clCount;
    double            cdSpiral;
    double            cdTumble;
    double            spinRadius_m;
};

struct RawMaterial {
    const char* name;
    const char* behaviour;
    double      density_kgm3;
    double      strength_Pa;
    double      toughness;
    double      thickness_m;
    double      elasticity;
    int         entersMedium;
};

extern const RawClassDefault kRawClassDefaults[];
extern const std::size_t     kRawClassDefaultCount;
extern const RawProjectile   kRawProjectiles[];
extern const std::size_t     kRawProjectileCount;
extern const RawBallProfile  kRawBallProfiles[];
extern const std::size_t     kRawBallProfileCount;
extern const RawMaterial     kRawMaterials[];
extern const std::size_t     kRawMaterialCount;

} // namespace pon::detail
