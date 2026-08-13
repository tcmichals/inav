# How-To Guide: BareCTF Binary Trace GUI Visualization & Analysis

> [!IMPORTANT]
> **OVERVIEW**
> In **`inav-abstractx`**, flight traces are recorded in **CTF (Common Trace Format)**—the Linux industry-standard open binary trace format.
>
> You can visualize CTF flight traces in **4 different GUI viewers** depending on your workflow!

---

## 1. GUI Option A: Betaflight / iNav Blackbox Explorer GUI (Recommended for PID Tuning)

If you prefer the familiar **Betaflight / iNav Blackbox Explorer** GUI to visualize PID loop responses, motor outputs, and 3D drone orientation:

### Step-by-Step Conversion:
```bash
# 1. Convert CTF 64B TLP binary log file to .BBL format
python3 tools/ctf_to_blackbox.py flight_log.tlp flight_log.bbl

# 2. Open flight_log.bbl in Betaflight Blackbox Explorer
```

---

## 2. GUI Option B: Eclipse Trace Compass GUI (Recommended for Deep System Debugging)

**Trace Compass** is the premier open-source GUI tool (developed by the Eclipse Foundation) for CTF trace analysis. It displays multi-stream 3D timelines, CPU frequency, nanosecond hardware timestamps, and FPGA DMA interrupt events.

### Step-by-Step Setup:
1. Download **Trace Compass**: `https://tracecompass.org/`
2. Open Trace Compass GUI.
3. Click `File` -> `Open Trace...` and select your CTF trace folder.
4. **Visual Features**:
   - Inspect nanosecond hardware timestamps (`timestamp_ns`).
   - Correlate flight controller PID updates directly with Linux SBC kernel events or FPGA DMA interrupts on the exact same timeline.

---

## 3. GUI Option C: Perfetto / Chrome Trace Viewer (`ui.perfetto.dev`)

Google **Perfetto** (`https://ui.perfetto.dev/`) and Chrome's built-in trace viewer (`chrome://tracing`) allow instant web browser trace visualization.

### Step-by-Step Setup:
```bash
# Convert CTF trace to JSON using babeltrace2
babeltrace2 ./log_folder --output-format=ctf-text > trace.txt
```
1. Open Google Chrome or Edge.
2. Navigate to `https://ui.perfetto.dev/` or `chrome://tracing`.
3. Drag and drop the trace file into the browser window to visualize the interactive timeline.

---

## 4. CLI Option D: `babeltrace2` Command Line Analyzer

For quick CLI inspection of CTF log records:

```bash
# Install babeltrace2 on Ubuntu/Debian
sudo apt install babeltrace2

# Inspect all CTF log records with nanosecond timestamps
babeltrace2 ./log_folder
```

---

## Summary of GUI Options

| GUI Tool | Purpose / Best For | Format |
| :--- | :--- | :--- |
| **Betaflight Blackbox Explorer** | PID tuning, motor outputs, 3D drone orientation | `.BBL` (via `ctf_to_blackbox.py`) |
| **Trace Compass** | Hardware nanosecond timelines, Linux kernel correlation | Native CTF folder |
| **Google Perfetto / Chrome** | Web browser interactive timeline inspection | JSON / CTF text |
| **PlotJuggler** | Robotics / ROS2 telemetry time-series plotting | CSV / CTF |
