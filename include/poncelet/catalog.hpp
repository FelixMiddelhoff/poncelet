// poncelet — named projectile catalog + material table (§3.1a / §4).
// SPDX-License-Identifier: MIT
//
// The shipped tables live in data/*.csv, baked to a C++ fragment at build time.
// `pon::catalog::get("9x19_124gr_fmj")` returns a fully-populated
// ProjectileType; the string id *is* the round. A game layers its own rows on
// top with load_csv() / add() (a later id replaces an earlier one).
#pragma once

#include "poncelet/projectile.hpp"
#include "poncelet/material.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace pon::catalog {

// Look up a named projectile. The returned type has every catalog field filled;
// Sim::registerType still applies class defaults to anything left blank. nullopt
// if `id` is unknown (check has() first if you want a hard failure).
std::optional<ProjectileType> find(std::string_view id);

// find(), but on an unknown id returns a bare Custom-class type with only `id`
// set — so registerType() falls back to Custom defaults rather than failing.
ProjectileType get(std::string_view id);

bool has(std::string_view id);

// Every known id (baked + overlay), sorted.
std::vector<std::string> ids();

// Runtime overlay. `path` / `text` use the same columns as data/projectiles.csv
// ('#' comment lines and blank lines ignored, first data line is the header).
// A row whose id already exists (baked or overlay) replaces it. Returns the
// number of rows applied, or -1 on a parse error (message into `err` if given).
int  load_csv(const std::string& path, std::string* err = nullptr);
int  load_csv_string(const std::string& text, std::string* err = nullptr);

// Add / replace a single entry.
void add(const ProjectileType& type);

// Drop every overlay row; back to the baked table.
void clear_overlay();

// Dump the current catalog (baked rows, with any overlay row replacing its
// baked counterpart by id) back to CSV text — the same column order
// data/projectiles.csv / bake_data.py use, sorted by id. There is no other
// way to get the shipped rounds back out; a dev who wants to tweak one
// otherwise has to transcribe it by hand. "export -> edit -> load_csv" is the
// intended authoring loop.
std::string to_csv();
// to_csv() written to `path` ('#' header comments are not reproduced — this
// is data, not the hand-authored file). Returns false (message into `err` if
// given) if `path` can't be opened for writing.
bool save_csv(const std::string& path, std::string* err = nullptr);

} // namespace pon::catalog

namespace pon::materials {

// Look up a shipped material by name (e.g. "oak", "concrete", "mild_steel").
// The returned Material's `name` points at storage owned by poncelet.
std::optional<Material> find(std::string_view name);

bool has(std::string_view name);
std::vector<std::string> names();

// Runtime overlay, same rules as pon::catalog::load_csv but with the columns of
// data/materials.csv.
int  load_csv(const std::string& path, std::string* err = nullptr);
int  load_csv_string(const std::string& text, std::string* err = nullptr);
void add(const Material& material);   // `material.name` is copied
void clear_overlay();

// As catalog::to_csv() / save_csv(), same rows and column order as
// data/materials.csv.
std::string to_csv();
bool save_csv(const std::string& path, std::string* err = nullptr);

} // namespace pon::materials
