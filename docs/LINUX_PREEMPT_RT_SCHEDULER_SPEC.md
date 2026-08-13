# Linux `PREEMPT_RT` Real-Time Kernel & Scheduler Hardening Specification

> [!IMPORTANT]
> **LINUX `SCHED_FIFO` VS `PREEMPT_RT` REAL-TIME KERNEL**
> While standard Linux provides POSIX real-time scheduling policies (`SCHED_FIFO` priorities 1--99), mainline Linux kernel preemption latencies can reach **$100\text{--}500\ \mu\text{s}$** due to non-preemptible kernel spinlocks.
>
> For an **8 kHz flight loop** ($125\ \mu\text{s}$ period), **`inav-abstractx`** requires the **Linux `PREEMPT_RT` kernel patch** combined with **`SCHED_FIFO` priority 99**, **CPU core isolation (`isolcpus=3`)**, and **physical memory locking (`mlockall`)** ([`linux_rt_hardener.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/sitl/linux_rt_hardener.hpp)).

---

## 1. Linux Scheduler Comparison Matrix

| Linux Kernel / Scheduler Policy | Worst-Case Preemption Latency | Suitable for 8 kHz Flight Loop ($125\ \mu\text{s}$)? | Technical Reason |
| :--- | :--- | :--- | :--- |
| **Standard Linux (`SCHED_OTHER`)** | $10\text{--}50\text{ ms}$ | **NO** | Completely Fair Scheduler (CFS) time-slicing |
| **Standard Linux (`SCHED_FIFO` Priority 99)** | $100\text{--}500\ \mu\text{s}$ | **NO (Occasional Deadline Misses)** | Non-preemptible kernel spinlocks & IRQ sections |
| **Linux `PREEMPT_RT` Patch (`SCHED_FIFO` 99)** | **$< 10\ \mu\text{s}$** | **YES (Hard Real-Time Guaranteed)** | Converts kernel spinlocks into preemptible mutexes |
| **`PREEMPT_RT` + `isolcpus=3` + `mlockall`** | **$< 2.5\ \mu\text{s}$** | **YES (Optimal Production Flight Target)** | Eliminates CPU interrupts and page fault stalls |

---

## 2. Recommended Kernel Boot Parameters (`/boot/cmdline.txt`)

To isolate CPU Core 3 for the Flight Control Loop on Allwinner Cubie A5E / Raspberry Pi / x86 SBC:

```bash
# Add to kernel boot command line:
isolcpus=3 nohz_full=3 rcu_nocbs=3 processor.max_cstate=1 intel_idle.max_cstate=0
```
