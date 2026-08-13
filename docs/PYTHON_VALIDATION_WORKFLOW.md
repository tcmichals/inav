# Python Automated Validation & File Output Workflow

This document outlines the procedures for outputting diagnostic files (`config.bin`, `flight_trace.ctf`, CSV logic traces) and running automated Python validation scripts to verify system behavior.

---

## 1. Automated Python Validation Tools (`tools/`)

| Script Name | Function & Purpose | Output File Generated / Parsed | Execution Command |
| :--- | :--- | :--- | :--- |
| **`run_all_validations.py`** | Master pipeline: Builds C++ executables, runs 9 unit test suites, executes hardware diagnostics, and checks math parity | Console log output | `python3 tools/run_all_validations.py` |
| **`compare_inav_parity.py`** | Differential math parity engine comparing legacy C vs C++20 flight math | Console parity report | `python3 tools/compare_inav_parity.py` |
| **`validate_logic_trace.py`** | Parses exported CSV logic analyzer traces (Saleae / Kingst) | Timing verification report | `python3 tools/validate_logic_trace.py trace.csv` |
| **`ctf_to_blackbox.py`** | Converts raw binary CTF flight traces into Betaflight/iNav `.BBL` log files | `flight_log.bbl` | `python3 tools/ctf_to_blackbox.py flight_trace.ctf` |

---

## 2. File Output Workflows

### A. Configuration Registry File (`config.bin`)
- **Generation**: Created automatically on first run of `inav_abstractx_sitl` or when executing `save` in iNav Configurator CLI.
- **Location**: `/home/tcmichals/ssdData/projects/home/inav/build/config.bin`.
- **Validation**: Verified by `test_config_and_flash()` in `./run_unit_tests`.

### B. CTF Binary Trace Stream (`flight_trace.ctf`)
- **Generation**: Streamed live over UDP port `19000` or saved to disk by `BlackboxLogger`.
- **Conversion**: Run `python3 tools/ctf_to_blackbox.py flight_trace.ctf` to generate `flight_log.bbl`.
- **GUI View**: Open `flight_log.bbl` in **Betaflight / iNav Blackbox Explorer** GUI.

---

## 3. One-Command Master Pipeline Execution

```bash
cd /home/tcmichals/ssdData/projects/home/inav
python3 tools/run_all_validations.py
```
