---
name: sitl-mission-review
description: >-
  Standardized protocol for iterative review, adversarial testing, and
  closed-loop 6-DOF SITL simulation verification to ensure mathematical and
  operational flight correctness.
---

# SITL Simulation & Multi-Pass Review Protocol

This skill enforces continuous, repeatable review and simulation verification across all flight control, sensor fusion, and navigation components.

## 1. The 4-Tier Verification Funnel

Every code change must pass all 4 tiers in succession:

```
+───────────────────────────────────────────────────────────────────────────+
| Tier 1: Static Analysis & MISRA C++ Safety Guards                         |
|   - Zero dynamic heap allocation (malloc/free/new/delete in flight path)  |
|   - NASA/JPL Power of 10 bounds checking, explicit static_cast<T>()       |
+─────────────────────────────────────┬─────────────────────────────────────+
                                      │
+─────────────────────────────────────▼─────────────────────────────────────+
| Tier 2: Submodule Differential Parity (Native C vs C++20)                 |
|   - Command: ./build/submodule_differential_test                          |
|   - Compares 1:1 against external/inav and external/betaflight            |
|   - Requirement: 100% bit-exact parity                                    |
+─────────────────────────────────────┬─────────────────────────────────────+
                                      │
+─────────────────────────────────────▼─────────────────────────────────────+
| Tier 3: Comprehensive Unit Tests & Benchmark Metrics                      |
|   - Commands: ./build/run_unit_tests && ./build/scheduler_benchmark       |
|   - All 20 CppUTest test groups passing                                   |
|   - 100,000-cycle scheduler benchmark verifying zero jitter/drift         |
+─────────────────────────────────────┬─────────────────────────────────────+
                                      │
+─────────────────────────────────────▼─────────────────────────────────────+
| Tier 4: Closed-Loop 6-DOF SITL Flight Mission Simulation                  |
|   - Command: ./build/full_stack_parity_test (60,000 flight loop ticks)    |
|   - Live TCP 5760 verification: python3 tools/test_sitl_integration.py    |
|   - Multi-phase mission: Takeoff -> Cruise -> S-Curve Turn -> RTH -> Land |
|   - Maximum allowable attitude error: 0.000000 deg                        |
+───────────────────────────────────────────────────────────────────────────+
```

## 2. Iterative SITL Execution Steps

```bash
# 1. Full compilation
mkdir -p build && cd build && cmake .. && make -j$(nproc)

# 2. Run automated validation runner (executes all tiers)
python3 tools/run_all_validations.py

# 3. Live SITL Interactive Session
./build/inav_abstractx_sitl &
SITL_PID=$!
sleep 1
python3 tools/test_sitl_integration.py
kill $SITL_PID 2>/dev/null
```

## 3. Adversarial Edge-Case Injection Checklist
When reviewing and testing new features in SITL, test:
- [ ] **Sensor Dropout**: Disconnect Baro or Mag mid-flight; assert failsafe dead-reckoning activates without attitude runaway.
- [ ] **RC Signal Loss**: Trigger failsafe stage 1 and stage 2; assert deterministic emergency descent or RTH activation.
- [ ] **Extreme Attitude Recovery**: Test flip recovery from $\pm 80^\circ$ pitch/roll in angle mode.
- [ ] **S-Curve Kinematics**: Verify deceleration stops at target waypoint without overshoot.
