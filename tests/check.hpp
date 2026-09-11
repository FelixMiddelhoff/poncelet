// poncelet — tiny zero-dependency test harness.
// SPDX-License-Identifier: MIT
#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

namespace pontest {

struct Registry {
    struct Case { const char* name; void (*fn)(); };
    std::vector<Case> cases;
    int failures = 0;
    static Registry& get() { static Registry r; return r; }
};

struct Reg {
    Reg(const char* name, void (*fn)()) { Registry::get().cases.push_back({name, fn}); }
};

inline void fail(const char* file, int line, const std::string& msg) {
    std::printf("  FAIL %s:%d  %s\n", file, line, msg.c_str());
    ++Registry::get().failures;
}

// Run every registered case, or — when `filter` is non-null — only the cases
// whose name contains it as a substring (used by the `poncelet_validation`
// ctest entry to run just the `validation_` suite).
inline int run_all(const char* filter = nullptr) {
    int failed_cases = 0, ran = 0;
    for (auto& c : Registry::get().cases) {
        if (filter && !std::strstr(c.name, filter)) continue;
        ++ran;
        const int before = Registry::get().failures;
        std::printf("[ RUN  ] %s\n", c.name);
        c.fn();
        if (Registry::get().failures > before) { std::printf("[ FAIL ] %s\n", c.name); ++failed_cases; }
        else                                    std::printf("[  OK  ] %s\n", c.name);
    }
    std::printf("\n%d cases, %d failed assertion(s), %d failed case(s)\n",
                ran, Registry::get().failures, failed_cases);
    return failed_cases == 0 ? 0 : 1;
}

} // namespace pontest

#define PON_TEST(name)                                                          \
    static void name();                                                         \
    static ::pontest::Reg pon_reg_##name(#name, &name);                         \
    static void name()

#define CHECK(cond)                                                             \
    do { if (!(cond)) ::pontest::fail(__FILE__, __LINE__, "CHECK(" #cond ")"); } while (0)

#define CHECK_NEAR(a, b, tol)                                                   \
    do {                                                                        \
        const double _a = (a), _b = (b), _t = (tol);                            \
        if (std::fabs(_a - _b) > _t) {                                          \
            char _buf[160];                                                     \
            std::snprintf(_buf, sizeof _buf,                                    \
                "CHECK_NEAR(" #a ", " #b ", " #tol ")  %.6g vs %.6g", _a, _b);  \
            ::pontest::fail(__FILE__, __LINE__, _buf);                          \
        }                                                                       \
    } while (0)
