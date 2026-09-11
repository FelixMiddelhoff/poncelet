# data/

The shipped default tables, as editable CSV. `tools/bake_data.py` bakes them to
`src/generated/data_tables.inc` (a plain-data C++ fragment, committed so a
Python-less build still works); `src/catalog.cpp` translates the rows into the
public `ProjectileType` / `Material` / internal `BallProfile`. The CMake build
re-bakes automatically when a Python 3 interpreter is available and a file here
changes; `ctest -R data_tables_baked_fresh` fails if the committed `.inc` is
stale.

| File | Reached through | Notes |
|---|---|---|
| `class_defaults.csv` | `Sim::registerType` (gap fill) | one row per `ProjectileClass` |
| `projectiles.csv` | `pon::catalog::get(id)` | the named catalog — the string id *is* the round |
| `ball_profiles.csv` | `ProjectileType::ballProfile` | per-ball `Cd` / `Cl` curves + shape |
| `materials.csv` | `pon::materials::find(name)` | terminal-ballistics seed properties |

Each file carries its sources in a `#` comment header. Values are seeds tuned to
the cited literature, not a validated fit — a game overrides any of them at
runtime with `pon::catalog::load_csv()` / `pon::materials::load_csv()` (a later
id / name wins). Format: `#` and blank lines ignored, first data line is the
header; curve columns are `x:y|x:y|...`.

The **G1 / G7 standard drag curves** are *not* in these CSVs — they are the
full-resolution BRL / Robert L. McCoy `Cd(Mach)` tabulations (JBM `mcg1.txt` /
`mcg7.txt`) hardcoded in `src/drag_tables.cpp`, and are reference data rather
than seeds. `projectiles.csv` rows just name `G1` / `G7` and carry a published
ballistic coefficient, which becomes the SI form factor scaling that curve.
