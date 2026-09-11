// poncelet — raylib sample: an aim-preview arc you can steer with the
// keyboard, drawn with raylib's 2D primitives. Uses pon::preview_arc
// directly (no Sim/World needed for a pure prediction line) — the same
// "poncelet owns no rendering, no geometry" boundary every other binding
// keeps; raylib only draws what preview_arc hands back.
// SPDX-License-Identifier: MIT
#include <poncelet/poncelet.hpp>

#include <raylib.h>

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

constexpr int kScreenW = 960;
constexpr int kScreenH = 540;
constexpr float kPixelsPerMetre = 3.5f; // downrange/height -> screen scale
constexpr float kGroundY = kScreenH - 60.0f; // muzzle height on screen

Vector2 WorldToScreen(pon::Real downrange_m, pon::Real height_m) {
    return {40.0f + static_cast<float>(downrange_m) * kPixelsPerMetre,
            kGroundY - static_cast<float>(height_m) * kPixelsPerMetre};
}

} // namespace

int main() {
    InitWindow(kScreenW, kScreenH, "poncelet + raylib — aim preview");
    SetTargetFPS(60);

    // A lobbed, draggy "shell" — same shape docs/examples/aim_preview.cpp
    // uses (a flat, fast rifle round travels farther than fits on screen
    // before it visibly arcs; this one draws a clear dome within a couple
    // hundred metres). The point here is the binding, not the catalog.
    pon::ProjectileType type;
    type.id = "raylib_demo";
    type.klass = pon::ProjectileClass::Shell;
    type.dragModel = pon::DragModel::ConstantCd;
    type.dragCoefficient = 0.30;
    type.mass_kg = 0.230;
    type.refDiameter_m = pon::inches(1.57); // 40 mm
    type.muzzleSpeed_mps = 76.0;

    const pon::Environment env; // ISA defaults

    float angleDeg = 25.0f;
    float speedMps = 76.0f;
    std::vector<pon::Vec3> arc;

    auto recompute = [&]() {
        pon::LaunchParams lp;
        lp.position = {0, 1.6, 0}; // standing shooter height, so it clears groundY=0
        const float a = angleDeg * (3.14159265358979323846f / 180.0f);
        lp.direction = {std::cos(a), std::sin(a), 0};
        lp.speed = speedMps;
        pon::preview_arc(type, lp, env, 0.01, 8.0, arc, /*groundY=*/0.0);
    };
    recompute();

    while (!WindowShouldClose()) {
        bool dirty = false;
        if (IsKeyDown(KEY_RIGHT) && angleDeg < 85.0f) { angleDeg += 30.0f * GetFrameTime(); dirty = true; }
        if (IsKeyDown(KEY_LEFT)  && angleDeg > 1.0f)  { angleDeg -= 30.0f * GetFrameTime(); dirty = true; }
        if (IsKeyDown(KEY_UP)    && speedMps < 200.0f) { speedMps += 40.0f * GetFrameTime(); dirty = true; }
        if (IsKeyDown(KEY_DOWN)  && speedMps > 10.0f)  { speedMps -= 40.0f * GetFrameTime(); dirty = true; }
        if (dirty) recompute();

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawLineEx(WorldToScreen(0, 0), WorldToScreen(1e6, 0), 2.0f, DARKGRAY); // ground

        for (std::size_t i = 1; i < arc.size(); ++i) {
            const Vector2 a = WorldToScreen(arc[i - 1].x, arc[i - 1].y);
            const Vector2 b = WorldToScreen(arc[i].x, arc[i].y);
            DrawLineEx(a, b, 2.0f, MAROON);
        }

        const double range_m = arc.empty() ? 0.0 : arc.back().x;
        char line1[96], line2[96];
        std::snprintf(line1, sizeof line1, "angle: %.0f deg   speed: %.0f m/s", angleDeg, speedMps);
        std::snprintf(line2, sizeof line2, "range: %.0f m   points: %zu", range_m, arc.size());
        DrawText(line1, 12, 12, 20, DARKGRAY);
        DrawText(line2, 12, 36, 20, DARKGRAY);
        DrawText("Left/Right: angle   Up/Down: speed", 12, kScreenH - 24, 18, GRAY);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
