# poncelet — vcpkg overlay port

```
vcpkg install poncelet --overlay-ports=<path-to-this-repo>/vcpkg-ports
```

Pinned to the `v1.0.0` tag with a checked SHA512 — this is now written in the
same format a real `microsoft/vcpkg` `ports/poncelet/` submission needs
(`vcpkg_from_github` with a fixed `REF`+`SHA512`, a `usage` file, `homepage`
in `vcpkg.json`).

**Not yet submitted to the central registry.** vcpkg's Maintainer Guide
requires a project be "mature" — a release ≥6 months old, or ≥6 months of
active public development — before a new-port PR will be accepted. poncelet's
public repo is from 2026-09-11; earliest reasonable submission date is
~2027-03. When that date arrives: re-derive the SHA512 against whatever tag
is current then (checksums are tag-specific, this one is stale the moment
`v1.0.0` moves), run `vcpkg x-add-version poncelet`, and open the PR as a
Draft against `microsoft/vcpkg` with this port copied into `ports/poncelet/`.

Static library only (`vcpkg_check_linkage(ONLY_STATIC_LIBRARY)`) — poncelet's
`PONCELET_SHARED` CMake option adds a second, un-aliased `poncelet_shared`
target meant for direct linking in-tree (see `bindings/unity`), not something
that maps cleanly onto vcpkg's per-triplet static/dynamic linkage model, so
it isn't exposed here.

**Verified**: `vcpkg install poncelet --overlay-ports=vcpkg-ports
--triplet x64-windows` builds both configs, generates the CMake package +
pkg-config, and passes vcpkg's post-build validation; a `find_package(poncelet)`
consumer configured against `vcpkg.cmake`'s toolchain file compiles, links,
and runs correctly against it.
