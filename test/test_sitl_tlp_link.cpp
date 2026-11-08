// Copyright (C) 2026 Tim Michals
// SPDX-License-Identifier: GPL-3.0-or-later
//
// @verifies [SPEC-SITL-03] docs/SITL_SIMULATOR.md: framing, routing, TCP_NODELAY, bytes unchanged,
// one client, disconnect and reconnect, steady state.

#include "sitl_tlp_link.hpp"

#include <cstdio>

namespace {

using abstractx::SpscTlpRing;
using abstractx::Tlp64;
using abstractx::target::sitl::SitlTlpLink;

uint32_t g_failures = 0u;

void check(bool ok, const char* what) {
    std::printf("%-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        ++g_failures;
    }
}

int connect_client(uint16_t port) {
    const int fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    (void)::inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    if (::connect(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) != 0) {
        ::close(fd);
        return -1;
    }
    const int one = 1;
    (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    return fd;
}

Tlp64 make(uint8_t type, uint8_t tag) {
    Tlp64 t{};
    t.wire.type = type;
    t.wire.tag = tag;
    t.wire.channel = ASP_CHANNEL_TELEMETRY;
    t.wire.target_address = 0x100Cu;
    t.wire.length_dw = 4u;
    t.wire.timestamp_ns = 1000000ull * tag;
    for (uint8_t i = 0u; i < 14u; ++i) {
        t.wire.payload[i] = static_cast<uint8_t>(tag + i);
    }
    return t;
}

void send_bytes(int fd, const uint8_t* p, size_t n) {
    while (n > 0u) {
        const ssize_t r = ::send(fd, p, n, MSG_NOSIGNAL);
        if (r <= 0) {
            return;
        }
        p += r;
        n -= static_cast<size_t>(r);
    }
}

bool recv_exact(int fd, uint8_t* p, size_t n) {
    while (n > 0u) {
        const ssize_t r = ::recv(fd, p, n, 0);
        if (r <= 0) {
            return false;
        }
        p += r;
        n -= static_cast<size_t>(r);
    }
    return true;
}

// Poll until `want` TLPs are routed or 200 polls pass.
size_t pump(SitlTlpLink& link, size_t want) {
    size_t got = 0u;
    for (int i = 0; i < 200 && got < want; ++i) {
        got += link.poll(5);
    }
    return got;
}

} // namespace

int main() {
    static SpscTlpRing<64u> stream;
    static SpscTlpRing<64u> completions;
    SitlTlpLink link{stream, completions};
    check(link.listen("127.0.0.1", 0u) && link.port() != 0u, "listens on a free port");

    int c = connect_client(link.port());
    check(c >= 0, "plugin connects");
    (void)link.poll(50);
    check(link.connected(), "connection accepted");

    // Nagle is off on INAV's side ([SPEC-SITL-03]); the client sets its own.
    check(link.nodelay(), "TCP_NODELAY set on the accepted socket");

    // Three TLPs (stream, completion, unexpected) sent split at odd places: 1, 63, 100, 28 bytes.
    {
        uint8_t buf[192];
        const Tlp64 a = make(ASP_TLP_TYPE_DMA_STREAM, 1u);
        const Tlp64 b = make(ASP_TLP_TYPE_CPL_D, 2u);
        const Tlp64 d = make(ASP_TLP_TYPE_MEM_WR, 3u);
        std::memcpy(buf, &a, 64);
        std::memcpy(buf + 64, &b, 64);
        std::memcpy(buf + 128, &d, 64);
        const size_t cuts[] = {1u, 63u, 100u, 28u};
        size_t off = 0u;
        size_t routed = 0u;
        for (size_t cut : cuts) {
            send_bytes(c, buf + off, cut);
            off += cut;
            routed += pump(link, 1u);
        }
        routed += pump(link, 3u - routed);
        check(routed == 3u, "3 TLPs reassembled from 4 uneven writes");
        const auto s = stream.pop();
        check(s && std::memcmp(&*s, &a, 60) == 0, "DMA_STREAM routed to the stream ring, intact");
        const auto p = completions.pop();
        check(p && p->wire.tag == 2u, "CPL_D routed to the completion ring");
        check(link.counters().unexpected_type == 1u, "MEM_WR from the plugin counted as unexpected");
        check(stream.empty() && completions.empty(), "nothing else delivered");
    }

    // Outbound: the 64 bytes arrive exactly as sent.
    {
        const Tlp64 m = make(ASP_TLP_TYPE_MEM_WR, 9u);
        check(link.send(m), "send to plugin");
        uint8_t got[64]{};
        check(recv_exact(c, got, 64u), "plugin receives 64 bytes");
        check(std::memcmp(got, &m, 64) == 0, "all 64 bytes unchanged");
    }

    // A second plugin is refused while one is connected.
    {
        const int c2 = connect_client(link.port());
        (void)link.poll(50);
        uint8_t x = 0u;
        const ssize_t r = (c2 >= 0) ? ::recv(c2, &x, 1u, 0) : 0;
        check(link.counters().refused_connections == 1u && r == 0, "second connection refused");
        if (c2 >= 0) {
            ::close(c2);
        }
    }

    // Disconnect in the middle of a TLP: the partial TLP is discarded, reconnect works.
    {
        const Tlp64 a = make(ASP_TLP_TYPE_DMA_STREAM, 4u);
        send_bytes(c, reinterpret_cast<const uint8_t*>(&a), 30u);
        (void)pump(link, 1u);
        ::close(c);
        for (int i = 0; i < 20 && link.connected(); ++i) {
            (void)link.poll(5);
        }
        check(!link.connected() && link.counters().disconnects == 1u, "disconnect detected");
        check(stream.empty(), "partial TLP discarded");
        c = connect_client(link.port());
        (void)link.poll(50);
        send_bytes(c, reinterpret_cast<const uint8_t*>(&a), 64u);
        check(pump(link, 1u) == 1u && stream.pop().has_value(), "reconnect delivers whole TLPs");
    }

    // Steady state: 1000 sample/command round trips, nothing grows or drops.
    {
        const uint64_t in0 = link.counters().tlps_in;
        bool ok = true;
        for (uint32_t i = 0u; i < 1000u && ok; ++i) {
            const Tlp64 s = make(ASP_TLP_TYPE_DMA_STREAM, static_cast<uint8_t>(i));
            send_bytes(c, reinterpret_cast<const uint8_t*>(&s), 64u);
            ok = pump(link, 1u) == 1u;
            const auto got = stream.pop();
            ok = ok && got && got->wire.tag == static_cast<uint8_t>(i) && stream.empty();
            ok = ok && link.send(make(ASP_TLP_TYPE_MEM_WR, static_cast<uint8_t>(i)));
            uint8_t back[64];
            ok = ok && recv_exact(c, back, 64u) && back[2] == static_cast<uint8_t>(i);
        }
        check(ok, "1000 round trips in order");
        check(link.counters().tlps_in - in0 == 1000u, "1000 TLPs in, counted");
        check(link.counters().dropped_ring_full == 0u, "no ring-full drops");
    }

    ::close(c);
    std::printf("%u failure(s)\n", g_failures);
    return (g_failures == 0u) ? 0 : 1;
}
