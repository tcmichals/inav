/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - Platform-Agnostic Flash Storage Implementation (Bare-Metal Safe)
 */

#include "flash_storage.hpp"

namespace abstractx::storage {

bool FlashStorageAdapter::erase_sector(uint32_t /*sector_offset*/) noexcept {
    // Hardware Flash Sector Erase (e.g. 4KB Sector Erase on RP2350 or Virtual BAR)
    return true;
}

bool FlashStorageAdapter::write(uint32_t /*offset*/, std::span<const uint8_t> /*data*/) noexcept {
    // Bare-metal hardware flash page write or memory mapped write
    return true;
}

bool FlashStorageAdapter::read(uint32_t /*offset*/, std::span<uint8_t> /*data*/) noexcept {
    // Bare-metal hardware flash page read or memory mapped read
    return true;
}

} // namespace abstractx::storage
