#!/usr/bin/env python3
"""
Copyright (C) 2026 Tim Michals
SPDX-License-Identifier: GPL-3.0-or-later

`inav-abstractx` - Comparative Flight Simulator: Legacy INAV Tasks vs C++20 Coroutines
"""

import sys
import math
import subprocess

def main():
    print("====================================================================")
    print(" FLIGHT EXECUTION BENCHMARK: LEGACY TASKS vs MODERN COROUTINES")
    print("====================================================================")

    res = subprocess.run(["./build/scheduler_benchmark"], capture_output=True, text=True)
    if res.returncode != 0:
        print(f"Error running benchmark: {res.stderr}")
        sys.exit(1)

    print(res.stdout)

if __name__ == "__main__":
    main()
