---
name: inav-parity-validation
description: >-
  Validation workflow for compiling inav-abstractx targets, executing the
  submodule-direct differential test suite against external/inav and external/betaflight,
  and validating zero-regression mathematical parity under MISRA C++ and DO-178C guidelines.
---

# INAV & Betaflight Parity Validation Skill

This skill defines the standardized procedure for compiling and running the differential parity test suite and unit tests against official INAV and Betaflight reference C source files.

## 1. Upstream Reference Locations

- **INAV Submodule**: [`external/inav/src/main/`](../../external/inav/src/main/)
- **Betaflight Submodule / Local Repo**: [`external/betaflight/src/main/`](../../external/betaflight/src/main/) or [`../betaflight/src/main/`](../../../betaflight/src/main/)
- **Master Plan**: [`docs/INAV_PORTING_MASTER_PLAN.md`](../../docs/INAV_PORTING_MASTER_PLAN.md)

## 2. Compilation and Test Execution Workflow

To build and run all test targets:

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
# 1. Native Differential Parity Suite (Direct 1:1 math comparison)
./submodule_differential_test
# 2. Comprehensive 20-Suite Unit Test Runner
./run_unit_tests
# 3. Full-Stack 60-Second Flight Mission Parity
./full_stack_parity_test
# 4. Coroutine Scheduler Benchmark
./scheduler_benchmark
```

## 3. Parity Rules & Quality Gates

1. **Zero Toy Stubs**: Never implement simplified approximations (e.g. unfiltered PID derivative or fixed-gain EKF). All filter stages, clamps, and state machines must match upstream reference C sources.
2. **Evidence-Based Completion**: All ported math must achieve 100% bit-exact parity or document numerical precision tolerances.
3. **Safety & Zero Dynamic Allocation**: Never introduce `malloc`, `new`, or unbounded recursion in flight code paths.
