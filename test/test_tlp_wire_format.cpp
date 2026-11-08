// Copyright (C) 2026 Tim Michals
// SPDX-License-Identifier: GPL-3.0-or-later
//
// @verifies [SPEC-SITL-02] docs/SITL_SIMULATOR.md: the Tlp64 INAV compiles against has
// AbstractX's wire layout and values, field by field against AbstractX's C struct.

#include "asp_tlp64.hpp"
#include "pcie_tlp_scheduler.hpp"
#include "tlp_channel.hpp"
#include "auto_dma_manager.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

namespace {

using Wire = decltype(abstractx::Tlp64{}.wire);

uint32_t g_failures = 0u;

void check(bool ok, const char* what, uint64_t got, uint64_t want) {
    std::printf("%-44s %s (got %llu, AbstractX %llu)\n", what, ok ? "ok  " : "FAIL",
                static_cast<unsigned long long>(got), static_cast<unsigned long long>(want));
    if (!ok) {
        ++g_failures;
    }
}

// Each argument is evaluated once (some are calls with side effects, e.g. await_resume()).
#define CHECK_EQ(what, got, want) \
    do { \
        const uint64_t got_v = static_cast<uint64_t>(got); \
        const uint64_t want_v = static_cast<uint64_t>(want); \
        check(got_v == want_v, what, got_v, want_v); \
    } while (0)

} // namespace

int main() {
    CHECK_EQ("sizeof(Tlp64)", sizeof(abstractx::Tlp64), sizeof(asp_tlp64_t));
    CHECK_EQ("offset type", offsetof(Wire, type), offsetof(asp_tlp64_t, type));
    CHECK_EQ("offset flags", offsetof(Wire, flags), offsetof(asp_tlp64_t, flags));
    CHECK_EQ("offset tag", offsetof(Wire, tag), offsetof(asp_tlp64_t, tag));
    CHECK_EQ("offset channel", offsetof(Wire, channel), offsetof(asp_tlp64_t, channel));
    CHECK_EQ("offset target_address", offsetof(Wire, target_address), offsetof(asp_tlp64_t, target_address));
    CHECK_EQ("offset length", offsetof(Wire, length), offsetof(asp_tlp64_t, length));
    CHECK_EQ("offset sequence", offsetof(Wire, sequence), offsetof(asp_tlp64_t, sequence));
    CHECK_EQ("offset timestamp_ns", offsetof(Wire, timestamp_ns), offsetof(asp_tlp64_t, timestamp_ns));
    CHECK_EQ("offset payload", offsetof(Wire, payload), offsetof(asp_tlp64_t, payload));
    CHECK_EQ("payload bytes", sizeof(Wire{}.payload), sizeof(asp_tlp64_t{}.payload));

    const auto wr = abstractx::Tlp64::make_mem_write(0x1000u, 0x12345678u, 7u);
    CHECK_EQ("make_mem_write type", wr.wire.type, ASP_TLP_TYPE_MEM_WR);
    CHECK_EQ("make_mem_write channel", wr.wire.channel, ASP_CHANNEL_CONTROL);
    const auto rd = abstractx::Tlp64::make_mem_read(0x1000u, 7u);
    CHECK_EQ("make_mem_read type", rd.wire.type, ASP_TLP_TYPE_MEM_RD);
    CHECK_EQ("make_mem_read channel", rd.wire.channel, ASP_CHANNEL_CONTROL);
    CHECK_EQ("TlpType::Completion", static_cast<uint8_t>(abstractx::TlpType::Completion), ASP_TLP_TYPE_CPL_D);
    CHECK_EQ("Channel::Telemetry", static_cast<uint8_t>(abstractx::Channel::Telemetry), ASP_CHANNEL_TELEMETRY);

    // A read longer than the payload must stop at the payload: bytes 60..63 are not INAV's.
    {
        static abstractx::SpscTlpRing<64u> out;
        static abstractx::SpscTlpRing<64u> in;
        abstractx::target::common::PcieTlpScheduler sched{out, in};
        for (uint8_t r = 40u; r < 44u; ++r) {
            sched.set_mock_reg(abstractx::bar::ImuBase, r, 0xAAu);
        }
        auto req = abstractx::Tlp64::make_mem_read(abstractx::bar::ImuBase, 9u);
        req.wire.length_dw = 15u; // 60 bytes, more than the 40-byte payload
        (void)out.push(req);
        (void)sched.process_outbound_queue();
        const auto comp = in.pop();
        CHECK_EQ("long read completes", comp.has_value(), 1u);
        if (comp) {
            std::array<uint8_t, 64> raw{};
            static_assert(sizeof(raw) == sizeof(comp->wire), "TLP size");
            __builtin_memcpy(raw.data(), &comp->wire, sizeof(raw));
            const uint32_t tail = static_cast<uint32_t>(raw[60]) | (static_cast<uint32_t>(raw[61]) << 8) |
                                  (static_cast<uint32_t>(raw[62]) << 16) | (static_cast<uint32_t>(raw[63]) << 24);
            CHECK_EQ("long read stays inside the payload", tail, 0u);
            CHECK_EQ("long read length <= payload", comp->length_bytes() <= sizeof(comp->wire.payload), 1u);
        }
    }

    // length is AbstractX's length_dw: 32-bit words, not bytes.
    {
        static abstractx::SpscTlpRing<64u> out;
        static abstractx::SpscTlpRing<64u> in;
        static abstractx::SpscTlpRing<64u> sched_out;
        abstractx::drivers::bus::TlpChannel chan{out, in};
        abstractx::target::common::PcieTlpScheduler sched{sched_out, in};
        for (uint8_t r = 0u; r < 16u; ++r) {
            sched.set_mock_reg(abstractx::bar::ImuBase, static_cast<uint8_t>(0x1Du + r), static_cast<uint8_t>(0x30u + r));
        }

        auto one = chan.async_read_reg(abstractx::bar::ImuBase, 0x1Du);
        one.await_suspend(std::coroutine_handle<>{});
        const auto one_req = out.pop();
        CHECK_EQ("1-byte read length_dw", one_req ? one_req->wire.length_dw : 0u, 1u);

        std::array<uint8_t, 14> burst{};
        auto aw = chan.async_read_burst(abstractx::bar::ImuBase, 0x1Du, std::span<uint8_t>(burst));
        aw.await_suspend(std::coroutine_handle<>{});
        const auto req = out.pop();
        CHECK_EQ("14-byte read length_dw", req ? req->wire.length_dw : 0u, 4u);
        if (req) {
            (void)sched_out.push(*req);
            (void)sched.process_outbound_queue();
            const auto comp = in.pop();
            CHECK_EQ("completion length_bytes", comp ? comp->length_bytes() : 0u, 16u);
            if (comp) {
                (void)in.push(*comp);
                CHECK_EQ("burst read resumes", aw.await_resume(), 1u);
                CHECK_EQ("burst byte 13", burst[13], 0x3Du);
            }
        }

        static abstractx::SpscTlpRing<64u> dma_ring;
        abstractx::target::common::AutoDmaManager dma{};
        const std::array<uint8_t, 14> sample{};
        (void)dma.register_channel(abstractx::target::common::DmaChannelType::ImuBurst, abstractx::bar::ImuBase, dma_ring);
        (void)dma.ingest_burst(abstractx::target::common::DmaChannelType::ImuBurst, std::span<const uint8_t>(sample), 1u);
        const auto dma_tlp = dma_ring.pop();
        CHECK_EQ("auto-DMA 14-byte burst length_dw", dma_tlp ? dma_tlp->wire.length_dw : 0u, 4u);
    }

    std::printf("%u failure(s)\n", g_failures);
    return (g_failures == 0u) ? 0 : 1;
}
