# Platform-Agnostic Flash Storage Abstraction API

> [!IMPORTANT]
> **STORAGE ABSTRACTION ARCHITECTURE**
> In **`inav-abstractx`**, configuration persistence (`MasterConfig`) is decoupled from physical storage hardware via the **`FlashStorageAdapter` API** ([`src/config/flash_storage.hpp`](file:///home/tcmichals/ssdData/projects/home/inav/src/config/flash_storage.hpp)).

---

## 1. Supported Storage Mediums (`FlashMediumType`)

1. **`PosixFile`**: Reads/writes POSIX binary file `config.bin` for Desktop SITL & HIL simulations.
2. **`Pico2OnChip`**: Reads/writes to Raspberry Pi RP2350 Pico 2 On-Chip SPI Flash sectors (`0x1F0000`).
3. **`LinuxMtdDevice`**: Reads/writes to Linux SBC Flash block / MTD devices (`/dev/mtd0`).
4. **`VirtualBar`**: Reads/writes over AbstractX Virtual BAR Flash storage space (`PCIE_BAR_SYS_BASE`).

---

## 2. C++20 Concept Definition (`IsFlashStorage`)

```cpp
template <typename T>
concept IsFlashStorage = requires(T storage, uint32_t offset, std::span<const uint8_t> wr_data, std::span<uint8_t> rd_data) {
    { storage.erase_sector(offset) } -> std::same_as<bool>;
    { storage.write(offset, wr_data) } -> std::same_as<bool>;
    { storage.read(offset, rd_data) } -> std::same_as<bool>;
};
```

---

## 3. Benefits

- **Zero Linker Section Hacks**: Completely eliminates legacy GCC custom linker script hacks (`.pg_registry` in `.ld` files).
- **Target Independence**: The exact same configuration registry (`ConfigRegistry::get()`) saves seamlessly on Pico 2 Flash, Linux MTD, Virtual BAR, and SITL files!
