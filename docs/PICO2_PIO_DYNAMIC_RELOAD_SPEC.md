# RP2350 Pico 2: Same-Pin Motor Signal Multiplexing Specification

> [!IMPORTANT]
> **EXACT SAME PHYSICAL MOTOR PINS (GPIO 2..5 / FPGA Motor Headers)**
> During normal flight, physical motor pins (`GPIO 2..5` on Pico 2 W, or FPGA I/O headers on SBC) act as **DShot Output Drivers**.
>
> When entering **BLHeli Suite 4-Way Passthrough Mode**, the **EXACT SAME PHYSICAL PINS** are dynamically re-assigned in PIO microcode to operate as **half-duplex 19200 baud 1-wire bidirectional serial UARTs** ([`pio_esc_reloader.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/target/pico2_rp2350/pio_esc_reloader.hpp))!

---

## 1. Same-Pin Hardware Multiplexing Diagram

```
                 Pico 2 W GPIO 2..5 / FPGA Motor Header Pin
                                    │
           ┌────────────────────────┴────────────────────────┐
           │                                                 │
           ▼                                                 ▼
   Flight Mode: DShot Output             4-Way Passthrough Mode: 1-Wire UART
 ┌───────────────────────────┐         ┌─────────────────────────────────────┐
 │ PIO0 Transmit-Only Engine │         │ PIO0 Half-Duplex 19200 Baud UART    │
 │ Outputs DShot150/300/600  │         │ Bidirectional 1-Wire Serial RX/TX   │
 │ digital motor frame pulses│         │ to flash 8-bit BLHeli ESC firmware  │
 └───────────────────────────┘         └─────────────────────────────────────┘
```

---

## 2. Microcode Hot-Swapping Sequence on the Same Pin

1. **Disable Active State Machine**: Stop PIO0 state machine on motor pin (`GPIO 2..5`).
2. **Unload Flight DShot Microcode**: `pio_remove_program(pio0, &pio_dshot_program)`.
3. **Load 1-Wire Serial Microcode**: `pio_add_program(pio0, &pio_blheli_4way_1wire_uart)`.
4. **Reconfigure Pin Direction**: Set PIO pin control to **open-drain half-duplex I/O** (`pio_sm_config_set_out_pins` and `pio_sm_config_set_in_pins` on the SAME pin).
5. **Re-enable State Machine**: Resume PIO state machine. The user can now flash/configure 8-bit BLHeli ESCs directly from **BLHeli Suite** without moving a single wire!
