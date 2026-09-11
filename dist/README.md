# dist/ — single-header poncelet

`poncelet_single.hpp` is the entire library (every header and `.cpp`, plus the
generated tables) amalgamated into one file by `tools/amalgamate.py`. **Generated
— do not edit.** Re-run `python3 tools/amalgamate.py` after changing a source;
the ctest `amalgamation_fresh` gate fails when it is out of date.

## Use

Vendor the one file. In **exactly one** translation unit:

```cpp
#define PONCELET_SINGLE_IMPLEMENTATION
#include "poncelet_single.hpp"
```

Every other translation unit just `#include "poncelet_single.hpp"`.

## Minified variant

`python3 tools/amalgamate.py --minify` additionally writes
`poncelet_single.min.hpp` — the same file with comments and blank lines
stripped (~40% smaller). Not committed and not freshness-checked; regenerate
it whenever you want it. Same usage as above, just a different filename.

## Determinism

The single-TU implementation build targets **`PlatformStable`** determinism. The
per-file `-ffp-contract=off` / `/fp:precise` flag the multi-file build puts on
`integrate.cpp` cannot be reproduced exactly in one TU (only an approximating
`#pragma` is emitted). For guaranteed cross-platform **`BitExact`**, use the
multi-file build, or compile the implementation TU with FP contraction and
fast-math disabled.
