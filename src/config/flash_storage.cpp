/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Flash Storage Implementation (Platform-Dispatched)
 *
 * PosixFile: Real fstream read/write for Linux SITL config.bin
 * Pico2OnChip: RP2350 flash_range_erase/program via Pico SDK (guarded by PICO_BOARD)
 */

#include "flash_storage.hpp"
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <fstream>
#endif

#if defined(PICO_BOARD)
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

// Flash target offset: last 4KB sector of 2MB flash (0x1FF000)
static constexpr uint32_t PICO_FLASH_TARGET_OFFSET = 0x1FF000;
#endif

namespace abstractx::storage {

bool FlashStorageAdapter::erase_sector(uint32_t sector_offset) noexcept {
    (void)sector_offset;
    switch (type_) {
        case FlashMediumType::PosixFile:
            // File overwrite handles erase implicitly
            return true;

#if defined(PICO_BOARD)
        case FlashMediumType::Pico2OnChip: {
            uint32_t ints = save_and_disable_interrupts();
            flash_range_erase(PICO_FLASH_TARGET_OFFSET + sector_offset, FLASH_SECTOR_SIZE);
            restore_interrupts(ints);
            return true;
        }
#endif

        case FlashMediumType::LinuxMtdDevice:
        case FlashMediumType::VirtualBar:
        default:
            return true; // Stub for unsupported medium types
    }
}

bool FlashStorageAdapter::write(uint32_t offset, std::span<const uint8_t> data) noexcept {
    switch (type_) {

#if defined(__linux__) || defined(__APPLE__)
        case FlashMediumType::PosixFile: {
            std::ofstream file("config.bin", std::ios::binary | std::ios::out);
            if (!file.is_open()) return false;
            if (offset > 0) {
                file.seekp(static_cast<std::streamoff>(offset));
            }
            file.write(reinterpret_cast<const char*>(data.data()),
                       static_cast<std::streamsize>(data.size()));
            return file.good();
        }
#endif

#if defined(PICO_BOARD)
        case FlashMediumType::Pico2OnChip: {
            // flash_range_program requires 256-byte aligned pages
            // Pad data to page boundary
            alignas(4) uint8_t page_buf[FLASH_PAGE_SIZE]{};
            size_t write_len = data.size();
            if (write_len > FLASH_PAGE_SIZE) write_len = FLASH_PAGE_SIZE;
            std::memcpy(page_buf, data.data(), write_len);

            uint32_t ints = save_and_disable_interrupts();
            flash_range_program(PICO_FLASH_TARGET_OFFSET + offset, page_buf,
                                (write_len + FLASH_PAGE_SIZE - 1) & ~(FLASH_PAGE_SIZE - 1));
            restore_interrupts(ints);
            return true;
        }
#endif

        default:
            return true;
    }
}

bool FlashStorageAdapter::read(uint32_t offset, std::span<uint8_t> data) noexcept {
    switch (type_) {

#if defined(__linux__) || defined(__APPLE__)
        case FlashMediumType::PosixFile: {
            std::ifstream file("config.bin", std::ios::binary | std::ios::in);
            if (!file.is_open()) {
                // File doesn't exist yet — fill with zeros (will trigger default config creation)
                std::memset(data.data(), 0, data.size());
                return false;
            }
            if (offset > 0) {
                file.seekg(static_cast<std::streamoff>(offset));
            }
            file.read(reinterpret_cast<char*>(data.data()),
                      static_cast<std::streamsize>(data.size()));
            return file.good();
        }
#endif

#if defined(PICO_BOARD)
        case FlashMediumType::Pico2OnChip: {
            // Direct XIP memory-mapped read — zero-copy!
            const uint8_t* flash_ptr = reinterpret_cast<const uint8_t*>(
                XIP_BASE + PICO_FLASH_TARGET_OFFSET + offset);
            std::memcpy(data.data(), flash_ptr, data.size());
            return true;
        }
#endif

        default:
            std::memset(data.data(), 0, data.size());
            return false;
    }
}

} // namespace abstractx::storage
