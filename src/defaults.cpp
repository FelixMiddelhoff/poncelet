// poncelet — class-default gap filling.
// SPDX-License-Identifier: MIT
//
// The class-default values live in data/class_defaults.csv (baked to
// src/generated/data_tables.inc); this file only applies them to the blank
// fields of a ProjectileType. The named catalog (<poncelet/catalog.hpp>) layers
// on top.
#include "defaults.hpp"

#include "catalog_internal.hpp"

namespace pon::detail {

bool apply_class_defaults(ProjectileType& t) {
    const ClassDefaults d = class_defaults_for(t.klass);

    if (t.refDiameter_m <= 0.0) t.refDiameter_m = d.refDiameter_m;
    if (t.mass_kg       <= 0.0) t.mass_kg       = d.mass_kg;

    // dragModel keeps whatever the caller set; only fill the coefficient the
    // active model needs.
    if (t.dragModel == DragModel::ConstantCd && !t.dragCoefficient)
        t.dragCoefficient = d.dragCoefficient > 0.0 ? d.dragCoefficient : 0.30;
    if ((t.dragModel == DragModel::G1 || t.dragModel == DragModel::G7) &&
        !t.ballisticCoefficient)
        t.ballisticCoefficient =
            d.ballisticCoefficient > 0.0 ? d.ballisticCoefficient : 0.30;

    if (!t.muzzleSpeed_mps) t.muzzleSpeed_mps = d.muzzleSpeed_mps;

    if (t.terminal.noseShapeFactor <= 0.0)
        t.terminal.noseShapeFactor = d.noseShapeFactor;

    return t.mass_kg > 0.0 && t.refDiameter_m > 0.0;
}

} // namespace pon::detail
