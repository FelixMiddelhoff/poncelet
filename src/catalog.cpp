// poncelet — named projectile catalog + material table.
// SPDX-License-Identifier: MIT
//
// Owns the baked tables (src/generated/data_tables.inc, from data/*.csv) and the
// runtime overlay. Translates the plain-data rows into the public
// ProjectileType / Material / (internal) BallProfile. Not FP-contract-off — it
// runs at content-load, never on the flight path.
#include "poncelet/catalog.hpp"

#include "catalog_data.hpp"
#include "catalog_internal.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <deque>
#include <fstream>
#include <sstream>
#include <unordered_map>

#include "generated/data_tables.inc"

namespace pon {
namespace {

// --- small string helpers --------------------------------------------------

std::string trim(std::string s) {
    auto notspace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notspace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notspace).base(), s.end());
    return s;
}

bool iequal(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
            return false;
    return true;
}

// --- enum <-> string -----------------------------------------------------

ProjectileClass parse_class(std::string_view s) {
    if (iequal(s, "Bullet"))      return ProjectileClass::Bullet;
    if (iequal(s, "Arrow"))       return ProjectileClass::Arrow;
    if (iequal(s, "Bolt"))        return ProjectileClass::Bolt;
    if (iequal(s, "Spear"))       return ProjectileClass::Spear;
    if (iequal(s, "ThrownBlade")) return ProjectileClass::ThrownBlade;
    if (iequal(s, "SportsBall"))  return ProjectileClass::SportsBall;
    if (iequal(s, "Pellet"))      return ProjectileClass::Pellet;
    if (iequal(s, "Shell"))       return ProjectileClass::Shell;
    if (iequal(s, "Rock"))        return ProjectileClass::Rock;
    return ProjectileClass::Custom;
}

DragModel parse_drag_model(std::string_view s) {
    if (iequal(s, "G1"))          return DragModel::G1;
    if (iequal(s, "G7"))          return DragModel::G7;
    if (iequal(s, "BallProfile")) return DragModel::BallProfile;
    if (iequal(s, "CustomCurve")) return DragModel::CustomCurve;
    return DragModel::ConstantCd;
}

SpinAxisMode parse_spin_axis(std::string_view s) {
    if (iequal(s, "Fixed")) return SpinAxisMode::Fixed;
    if (iequal(s, "Free"))  return SpinAxisMode::Free;
    return SpinAxisMode::AlongVelocity;
}

BallShape parse_ball_shape(std::string_view s) {
    if (iequal(s, "ProlateSpheroid")) return BallShape::ProlateSpheroid;
    if (iequal(s, "Disc"))            return BallShape::Disc;
    return BallShape::Sphere;
}

MaterialBehaviour parse_behaviour(std::string_view s) {
    if (iequal(s, "Brittle"))  return MaterialBehaviour::Brittle;
    if (iequal(s, "Ductile"))  return MaterialBehaviour::Ductile;
    if (iequal(s, "Fibrous"))  return MaterialBehaviour::Fibrous;
    if (iequal(s, "Membrane")) return MaterialBehaviour::Membrane;
    if (iequal(s, "Granular")) return MaterialBehaviour::Granular;
    return MaterialBehaviour::Fluid;
}

// --- enum -> string (Tier D3: catalog::to_csv() / materials::to_csv()) -----

const char* class_name(ProjectileClass c) {
    switch (c) {
        case ProjectileClass::Bullet:      return "Bullet";
        case ProjectileClass::Arrow:       return "Arrow";
        case ProjectileClass::Bolt:        return "Bolt";
        case ProjectileClass::Spear:       return "Spear";
        case ProjectileClass::ThrownBlade: return "ThrownBlade";
        case ProjectileClass::SportsBall:  return "SportsBall";
        case ProjectileClass::Pellet:      return "Pellet";
        case ProjectileClass::Shell:       return "Shell";
        case ProjectileClass::Rock:        return "Rock";
        case ProjectileClass::Custom:      return "Custom";
    }
    return "Custom";
}

const char* drag_model_name(DragModel d) {
    switch (d) {
        case DragModel::G1:          return "G1";
        case DragModel::G7:          return "G7";
        case DragModel::BallProfile: return "BallProfile";
        case DragModel::CustomCurve: return "CustomCurve";
        case DragModel::ConstantCd:  return "ConstantCd";
    }
    return "ConstantCd";
}

const char* spin_axis_name(SpinAxisMode m) {
    switch (m) {
        case SpinAxisMode::Fixed:         return "Fixed";
        case SpinAxisMode::Free:          return "Free";
        case SpinAxisMode::AlongVelocity: return "AlongVelocity";
    }
    return "AlongVelocity";
}

const char* behaviour_name(MaterialBehaviour b) {
    switch (b) {
        case MaterialBehaviour::Brittle:  return "Brittle";
        case MaterialBehaviour::Ductile:  return "Ductile";
        case MaterialBehaviour::Fibrous:  return "Fibrous";
        case MaterialBehaviour::Membrane: return "Membrane";
        case MaterialBehaviour::Granular: return "Granular";
        case MaterialBehaviour::Fluid:    return "Fluid";
    }
    return "Fluid";
}

// Shortest round-trippable text for a CSV cell — matches the plain decimal
// style data/*.csv is hand-authored in (no scientific notation for the
// ranges these fields take).
std::string csv_num(double v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%g", v);
    return buf;
}

// --- raw-row -> public/internal struct ----------------------------------

ProjectileType from_raw(const detail::RawProjectile& r) {
    ProjectileType t;
    t.id        = r.id;
    t.klass     = parse_class(r.klass);
    t.dragModel = parse_drag_model(r.dragModel);
    if (r.mass_kg > 0.0)       t.mass_kg       = r.mass_kg;
    if (r.refDiameter_m > 0.0) t.refDiameter_m = r.refDiameter_m;
    if (r.ballisticCoefficient > 0.0) t.ballisticCoefficient = r.ballisticCoefficient;
    if (r.dragCoefficient > 0.0)      t.dragCoefficient      = r.dragCoefficient;
    if (r.ballProfile && r.ballProfile[0]) t.ballProfile = r.ballProfile;
    if (r.muzzleSpeed_mps > 0.0) t.muzzleSpeed_mps = r.muzzleSpeed_mps;

    t.spinAxisMode = parse_spin_axis(r.spinAxisMode);
    t.twistRate_m  = r.twistRate_m;
    if (r.spinRate_radps > 0.0) t.spinRate_radps = r.spinRate_radps;

    if (r.noseShapeFactor > 0.0) t.terminal.noseShapeFactor = r.noseShapeFactor;
    if (r.hardness > 0.0)        t.terminal.hardness         = r.hardness;
    t.terminal.deformable = r.deformable != 0;
    t.terminal.fragile    = r.fragile != 0;
    return t;
}

Material from_raw(const detail::RawMaterial& r) {
    Material m;
    m.name         = r.name;       // string literal from the baked .inc
    m.behaviour    = parse_behaviour(r.behaviour);
    m.density_kgm3 = r.density_kgm3;
    m.strength_Pa  = r.strength_Pa;
    m.toughness    = r.toughness;
    m.thickness_m  = r.thickness_m;
    m.elasticity   = r.elasticity;
    m.entersMedium = r.entersMedium;
    return m;
}

// --- CSV parsing --------------------------------------------------------

struct Csv {
    std::vector<std::string>              header;
    std::vector<std::vector<std::string>> rows;
    std::string                           error;
};

Csv parse_csv(const std::string& text) {
    Csv out;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::string t = trim(line);
        if (t.empty() || t[0] == '#') continue;
        std::vector<std::string> cells;
        std::string cell;
        std::istringstream ls(line);
        while (std::getline(ls, cell, ',')) cells.push_back(trim(cell));
        if (line.size() && line.back() == ',') cells.push_back("");
        if (out.header.empty()) out.header = std::move(cells);
        else                    out.rows.push_back(std::move(cells));
    }
    if (out.header.empty()) out.error = "no header row";
    return out;
}

int col_of(const std::vector<std::string>& header, const char* name) {
    for (std::size_t i = 0; i < header.size(); ++i)
        if (header[i] == name) return (int)i;
    return -1;
}

// Field accessor bound to one row + header.
struct Row {
    const std::vector<std::string>* header;
    const std::vector<std::string>* cells;
    std::string str(const char* c) const {
        int i = col_of(*header, c);
        return (i >= 0 && (std::size_t)i < cells->size()) ? (*cells)[i] : std::string();
    }
    double num(const char* c) const {
        std::string s = str(c);
        if (s.empty()) return 0.0;
        try { return std::stod(s); } catch (...) { return 0.0; }
    }
    int integer(const char* c) const { return (int)num(c); }
};

// --- catalog state ----------------------------------------------------

struct CatalogState {
    // baked, translated once
    std::vector<ProjectileType>                  baked;
    std::unordered_map<std::string, std::size_t> bakedIndex;
    // overlay (later add() replaces earlier)
    std::vector<ProjectileType>                  overlay;
    std::unordered_map<std::string, std::size_t> overlayIndex;

    CatalogState() {
        baked.reserve(detail::kRawProjectileCount);
        for (std::size_t i = 0; i < detail::kRawProjectileCount; ++i) {
            baked.push_back(from_raw(detail::kRawProjectiles[i]));
            bakedIndex[baked.back().id] = baked.size() - 1;
        }
    }

    const ProjectileType* lookup(std::string_view id) const {
        std::string key(id);
        auto o = overlayIndex.find(key);
        if (o != overlayIndex.end()) return &overlay[o->second];
        auto b = bakedIndex.find(key);
        if (b != bakedIndex.end()) return &baked[b->second];
        return nullptr;
    }

    void put(const ProjectileType& t) {
        auto it = overlayIndex.find(t.id);
        if (it != overlayIndex.end()) overlay[it->second] = t;
        else { overlayIndex[t.id] = overlay.size(); overlay.push_back(t); }
    }
};

CatalogState& cat() { static CatalogState s; return s; }

ProjectileType projectile_from_row(const Row& r) {
    detail::RawProjectile raw{};
    std::string id = r.str("id"), klass = r.str("klass"), dm = r.str("drag_model"),
                bp = r.str("ball_profile"), sam = r.str("spin_axis_mode");
    raw.id           = id.c_str();
    raw.klass        = klass.c_str();
    raw.dragModel    = dm.empty() ? "ConstantCd" : dm.c_str();
    raw.mass_kg      = r.num("mass_kg");
    raw.refDiameter_m = r.num("ref_diameter_m");
    raw.ballisticCoefficient = r.num("ballistic_coefficient");
    raw.dragCoefficient      = r.num("drag_coefficient");
    raw.ballProfile  = bp.c_str();
    raw.muzzleSpeed_mps = r.num("muzzle_speed_mps");
    raw.noseShapeFactor = r.num("nose_shape_factor");
    raw.hardness     = r.num("hardness");
    raw.deformable   = r.integer("deformable");
    raw.fragile      = r.integer("fragile");
    raw.spinRate_radps = r.num("spin_rate_radps");
    raw.spinAxisMode = sam.empty() ? "AlongVelocity" : sam.c_str();
    raw.twistRate_m  = r.num("twist_rate_m");
    return from_raw(raw);   // copies out of the c_str()s before they die
}

// --- material state --------------------------------------------------

struct MaterialState {
    std::vector<Material>                        baked;
    std::unordered_map<std::string, std::size_t> bakedIndex;
    std::deque<std::string>                      names;   // stable storage
    std::vector<Material>                        overlay;
    std::unordered_map<std::string, std::size_t> overlayIndex;

    MaterialState() {
        for (std::size_t i = 0; i < detail::kRawMaterialCount; ++i) {
            baked.push_back(from_raw(detail::kRawMaterials[i]));
            bakedIndex[baked.back().name] = baked.size() - 1;
        }
    }

    const Material* lookup(std::string_view name) const {
        std::string key(name);
        auto o = overlayIndex.find(key);
        if (o != overlayIndex.end()) return &overlay[o->second];
        auto b = bakedIndex.find(key);
        if (b != bakedIndex.end()) return &baked[b->second];
        return nullptr;
    }

    void put(Material m, const std::string& name) {
        names.push_back(name);
        m.name = names.back().c_str();
        auto it = overlayIndex.find(name);
        if (it != overlayIndex.end()) overlay[it->second] = m;
        else { overlayIndex[name] = overlay.size(); overlay.push_back(m); }
    }
};

MaterialState& mats() { static MaterialState s; return s; }

Material material_from_row(const Row& r) {
    Material m;
    m.behaviour    = parse_behaviour(r.str("behaviour"));
    m.density_kgm3 = r.num("density_kgm3");
    m.strength_Pa  = r.num("strength_pa");
    m.toughness    = r.num("toughness_j_m2");
    m.thickness_m  = r.num("thickness_m");
    m.elasticity   = r.num("elasticity");
    int em = col_of(*r.header, "enters_medium");
    m.entersMedium = (em >= 0) ? r.integer("enters_medium") : -1;
    return m;
}

} // namespace

// ==========================================================================
// pon::catalog
// ==========================================================================
namespace catalog {

std::optional<ProjectileType> find(std::string_view id) {
    if (const ProjectileType* t = cat().lookup(id)) return *t;
    return std::nullopt;
}

ProjectileType get(std::string_view id) {
    if (const ProjectileType* t = cat().lookup(id)) return *t;
    ProjectileType t;
    t.id = std::string(id);
    t.klass = ProjectileClass::Custom;
    return t;
}

bool has(std::string_view id) { return cat().lookup(id) != nullptr; }

std::vector<std::string> ids() {
    std::vector<std::string> out;
    for (const auto& t : cat().baked)   out.push_back(t.id);
    for (const auto& t : cat().overlay)
        if (cat().bakedIndex.find(t.id) == cat().bakedIndex.end())
            out.push_back(t.id);
    std::sort(out.begin(), out.end());
    return out;
}

int load_csv_string(const std::string& text, std::string* err) {
    Csv csv = parse_csv(text);
    if (!csv.error.empty()) { if (err) *err = csv.error; return -1; }
    if (col_of(csv.header, "id") < 0) {
        if (err) *err = "projectiles CSV needs an 'id' column";
        return -1;
    }
    int n = 0;
    for (const auto& cells : csv.rows) {
        Row row{&csv.header, &cells};
        if (row.str("id").empty()) continue;
        cat().put(projectile_from_row(row));
        ++n;
    }
    return n;
}

int load_csv(const std::string& path, std::string* err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) { if (err) *err = "cannot open " + path; return -1; }
    std::stringstream ss;
    ss << in.rdbuf();
    return load_csv_string(ss.str(), err);
}

void add(const ProjectileType& type) { cat().put(type); }
void clear_overlay() { cat().overlay.clear(); cat().overlayIndex.clear(); }

std::string to_csv() {
    std::string out =
        "id,klass,drag_model,mass_kg,ref_diameter_m,ballistic_coefficient,"
        "drag_coefficient,ball_profile,muzzle_speed_mps,nose_shape_factor,"
        "hardness,deformable,fragile,spin_rate_radps,spin_axis_mode,twist_rate_m\n";
    for (const std::string& id : ids()) {
        const ProjectileType t = get(id); // known id -> always resolves
        out += id; out += ',';
        out += class_name(t.klass); out += ',';
        out += drag_model_name(t.dragModel); out += ',';
        out += csv_num(t.mass_kg); out += ',';
        out += csv_num(t.refDiameter_m); out += ',';
        out += csv_num(t.ballisticCoefficient.value_or(0.0)); out += ',';
        out += csv_num(t.dragCoefficient.value_or(0.0)); out += ',';
        out += t.ballProfile; out += ',';
        out += csv_num(t.muzzleSpeed_mps.value_or(0.0)); out += ',';
        out += csv_num(t.terminal.noseShapeFactor); out += ',';
        out += csv_num(t.terminal.hardness); out += ',';
        out += (t.terminal.deformable ? '1' : '0'); out += ',';
        out += (t.terminal.fragile ? '1' : '0'); out += ',';
        out += csv_num(t.spinRate_radps.value_or(0.0)); out += ',';
        out += spin_axis_name(t.spinAxisMode); out += ',';
        out += csv_num(t.twistRate_m);
        out += '\n';
    }
    return out;
}

bool save_csv(const std::string& path, std::string* err) {
    std::ofstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "cannot open " + path; return false; }
    const std::string text = to_csv();
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    return f.good();
}

} // namespace catalog

// ==========================================================================
// pon::materials
// ==========================================================================
namespace materials {

std::optional<Material> find(std::string_view name) {
    if (const Material* m = mats().lookup(name)) return *m;
    return std::nullopt;
}

bool has(std::string_view name) { return mats().lookup(name) != nullptr; }

std::vector<std::string> names() {
    std::vector<std::string> out;
    for (const auto& m : mats().baked)   out.push_back(m.name);
    for (const auto& kv : mats().overlayIndex)
        if (mats().bakedIndex.find(kv.first) == mats().bakedIndex.end())
            out.push_back(kv.first);
    std::sort(out.begin(), out.end());
    return out;
}

int load_csv_string(const std::string& text, std::string* err) {
    Csv csv = parse_csv(text);
    if (!csv.error.empty()) { if (err) *err = csv.error; return -1; }
    if (col_of(csv.header, "name") < 0) {
        if (err) *err = "materials CSV needs a 'name' column";
        return -1;
    }
    int n = 0;
    for (const auto& cells : csv.rows) {
        Row row{&csv.header, &cells};
        std::string nm = row.str("name");
        if (nm.empty()) continue;
        mats().put(material_from_row(row), nm);
        ++n;
    }
    return n;
}

int load_csv(const std::string& path, std::string* err) {
    std::ifstream in(path, std::ios::binary);
    if (!in) { if (err) *err = "cannot open " + path; return -1; }
    std::stringstream ss;
    ss << in.rdbuf();
    return load_csv_string(ss.str(), err);
}

void add(const Material& material) {
    mats().put(material, material.name ? material.name : "unnamed");
}
void clear_overlay() {
    mats().overlay.clear();
    mats().overlayIndex.clear();
}

std::string to_csv() {
    std::string out =
        "name,behaviour,density_kgm3,strength_pa,toughness_j_m2,thickness_m,"
        "elasticity,enters_medium\n";
    for (const std::string& name : names()) {
        const Material m = *find(name); // known name -> always resolves
        out += name; out += ',';
        out += behaviour_name(m.behaviour); out += ',';
        out += csv_num(m.density_kgm3); out += ',';
        out += csv_num(m.strength_Pa); out += ',';
        out += csv_num(m.toughness); out += ',';
        out += csv_num(m.thickness_m); out += ',';
        out += csv_num(m.elasticity); out += ',';
        out += std::to_string(m.entersMedium);
        out += '\n';
    }
    return out;
}

bool save_csv(const std::string& path, std::string* err) {
    std::ofstream f(path, std::ios::binary);
    if (!f) { if (err) *err = "cannot open " + path; return false; }
    const std::string text = to_csv();
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    return f.good();
}

} // namespace materials

// ==========================================================================
// internal accessors (catalog_internal.hpp)
// ==========================================================================
namespace detail {

ClassDefaults class_defaults_for(ProjectileClass k) {
    const char* want = nullptr;
    switch (k) {
        case ProjectileClass::Bullet:      want = "Bullet"; break;
        case ProjectileClass::Arrow:       want = "Arrow"; break;
        case ProjectileClass::Bolt:        want = "Bolt"; break;
        case ProjectileClass::Spear:       want = "Spear"; break;
        case ProjectileClass::ThrownBlade: want = "ThrownBlade"; break;
        case ProjectileClass::SportsBall:  want = "SportsBall"; break;
        case ProjectileClass::Pellet:      want = "Pellet"; break;
        case ProjectileClass::Shell:       want = "Shell"; break;
        case ProjectileClass::Rock:        want = "Rock"; break;
        case ProjectileClass::Custom:      want = "Custom"; break;
    }
    ClassDefaults d;
    for (std::size_t i = 0; i < kRawClassDefaultCount; ++i) {
        const RawClassDefault& r = kRawClassDefaults[i];
        if (want && std::string_view(r.klass) == want) {
            d.refDiameter_m        = r.refDiameter_m;
            d.mass_kg              = r.mass_kg;
            d.dragModel            = parse_drag_model(r.dragModel);
            d.dragCoefficient      = r.dragCoefficient;
            d.ballisticCoefficient = r.ballisticCoefficient;
            d.muzzleSpeed_mps      = r.muzzleSpeed_mps;
            d.noseShapeFactor      = r.noseShapeFactor;
            break;
        }
    }
    return d;
}

const std::vector<BallProfile>& baked_ball_profiles() {
    static const std::vector<BallProfile> table = [] {
        std::vector<BallProfile> t;
        t.reserve(kRawBallProfileCount);
        for (std::size_t i = 0; i < kRawBallProfileCount; ++i) {
            const RawBallProfile& r = kRawBallProfiles[i];
            BallProfile p;
            p.id               = r.id;
            p.shape            = parse_ball_shape(r.shape);
            p.diameter_m       = r.diameter_m;
            p.mass_kg          = r.mass_kg;
            p.typicalSpeed_mps = r.typicalSpeed_mps;
            p.abscissa         = std::string_view(r.abscissa) == "Mach"
                                     ? DragAbscissa::Mach : DragAbscissa::Reynolds;
            for (std::size_t k = 0; k < r.cdCount; ++k)
                p.cdCurve.push_back({r.cdCurve[k].x, r.cdCurve[k].y});
            for (std::size_t k = 0; k < r.clCount; ++k)
                p.clCurve.push_back({r.clCurve[k].x, r.clCurve[k].y});
            p.cdSpiral     = r.cdSpiral;
            p.cdTumble     = r.cdTumble;
            p.spinRadius_m = r.spinRadius_m;
            t.push_back(std::move(p));
        }
        return t;
    }();
    return table;
}

} // namespace detail
} // namespace pon
