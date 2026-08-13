#!/usr/bin/env python3
"""
Copyright (C) 2026 Tim Michals
SPDX-License-Identifier: GPL-3.0-or-later

`tcmichals/inav` - Master Automated System Validation & Test Pipeline
"""

import os
import sys
import subprocess

def run_step(description, command):
    print(f"\n====================================================")
    print(f" EXECUTION STEP: {description}")
    print(f" Command: {command}")
    print(f"====================================================")
    
    result = subprocess.run(command, shell=True)
    if result.returncode != 0:
        print(f"FAILED: Step '{description}' failed with exit code {result.returncode}")
        return False
    print(f"PASSED: Step '{description}' completed successfully!")
    return True

def main():
    print("====================================================")
    print(" STARTING INAV-ABSTRACTX MASTER VALIDATION PIPELINE")
    print("====================================================")

    # 1. Build CMake Executables
    if not run_step("Compile C++20 Executables", "mkdir -p build && cd build && cmake .. && cmake --build ."):
        sys.exit(1)

    # 2. Execute 9-Suite Unit Tests
    if not run_step("Execute 9-Suite C++20 Unit Tests", "cd build && ./run_unit_tests"):
        sys.exit(1)

    # 3. Execute On-Device Hardware Test Harness
    if not run_step("Execute Hardware Diagnostic Harness", "cd build && ./pico2_hw_test"):
        sys.exit(1)

    # 4. Execute 4-Subsystem Differential Parity Test
    if not run_step("Execute Differential Math Parity Test", "python3 tools/compare_inav_parity.py"):
        sys.exit(1)

    # 5. Execute Logic Analyzer Trace Validator
    if not run_step("Execute Offline Logic Analyzer Trace Validator", "python3 tools/validate_logic_trace.py"):
        sys.exit(1)

    print("\n====================================================")
    print(" ALL SYSTEM VALIDATION PIPELINES PASSED 100%!")
    print("====================================================")

if __name__ == "__main__":
    main()
