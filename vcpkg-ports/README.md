# poncelet — vcpkg overlay port

```
vcpkg install poncelet --overlay-ports=<path-to-this-repo>/vcpkg-ports
```

Works today, no public remote required — see `poncelet/portfile.cmake`'s
header comment for how (a local `file://` git clone of this checkout, until
poncelet has a real remote + tagged release to point at instead).

This is an **overlay port**, not a submission to the central vcpkg registry
(`microsoft/vcpkg`) — that's a separate step (a PR there, their own CI/review),
not doable without a real GitHub remote and a maintainer identity to submit
under.

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
