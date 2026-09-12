# poncelet — Conan Center Index recipe (staged)

This mirrors the exact directory layout a `conan-io/conan-center-index`
submission needs: `recipes/poncelet/config.yml` +
`recipes/poncelet/all/{conanfile.py,conandata.yml,test_package/}`. It's
different from the root `conanfile.py` in this repo, which is a local/overlay
recipe (`exports_sources` packages the checkout directly via `conan create .`,
no registry involved) — the registry format here fetches a checksummed
source tarball via `conandata.yml` instead, since CCI recipes must always
match a real tagged upstream release, never a local working copy.

**Verified locally** (2026-09-12): `conan create . --version 1.0.0
-s compiler.cppstd=17 --build=missing` from `all/` — downloads the pinned
`v1.0.0` tarball, checksum matches, builds the static `poncelet::poncelet`
CMake target, packages it with the LICENSE, and `test_package` (a real
`find_package(poncelet)` consumer) compiles, links, and runs against the
packaged library.

To submit: fork `conan-io/conan-center-index`, copy this directory's
contents to `recipes/poncelet/` in that fork, open a PR. No project-maturity
gate like vcpkg's, but expect the standard CCI review process: automated
CI across 30+ configs, two maintainer approvals to merge, and a real
backlog for new recipes — weeks, not days, is normal.

If `v1.0.0` ever moves (owner's policy: it stays fixed — see repo memory),
this recipe doesn't need touching; if a genuinely new version is ever
tagged, add a new entry to `config.yml` and `conandata.yml` rather than
editing the `1.0.0` one.
