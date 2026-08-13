/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Platform-Agnostic Flash Storage Abstraction API
 */

#ifndef FLASH_STORAGE_HPP
#define FLASH_STORAGE_HPP

#include <cstdint>
#include <cstddef>
#include <span>

namespace abstractx::storage {

// Flash Storage Medium Type
enum class FlashMediumType : uint8_t {
    PosixFile     = 0, // Linux Desktop SITL config.bin file
    Pico2OnChip   = 1, // RP2350 Pico 2 On-Chip Flash sector (0x1F0000)
    LinuxMtdDevice = 2, // Linux SBC Flash Block / MTD device (/dev/mtd0)
    VirtualBar    = 3  // AbstractX Virtual BAR Flash storage space
};

// Platform-Agnostic Flash Storage API Concept
template <typename T>
concept IsFlashStorage = requires(T storage, uint32_t offset, std::span<const uint8_t> wr_data, std::span<uint8_t> rd_data) {
    { storage.erase_sector(offset) } -> std::same_as<bool>;
    { storage.write(offset, wr_data) } -> std::same_as<bool>;
    { storage.read(offset, rd_data) } -> std::same_as<bool>;
};

// Generic Flash Storage Adapter
class FlashStorageAdapter {
public:
    constexpr explicit FlashStorageAdapter(FlashMediumType type = FlashMediumType::PosixFile) noexcept 
        : type_(type) {}

    // Erase 4KB Flash Sector
    bool erase_sector(uint32_t sector_offset) noexcept;

    // Write binary bytes to Flash memory
    bool write(uint32_t offset, std::span<const uint8_t> data) noexcept;

    // Read binary bytes from Flash memory
    bool read(uint32_t offset, std::span<uint8_t> data) noexcept;

    constexpr FlashMediumType type() const noexcept { return type_; }

private:
    FlashMediumType type_{FlashMediumType::PosixFile};
};

} // namespace abstractx::storage

#endif // FLASH_STORAGE_HPP
