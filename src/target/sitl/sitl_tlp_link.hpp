// Copyright (C) 2026 Tim Michals
// SPDX-License-Identifier: GPL-3.0-or-later
//
// SITL TLP link: whole 64-byte TLPs over TCP between the SITL binary and the Studio plugin.
// @impl [SPEC-SITL-03] docs/SITL_SIMULATOR.md

#ifndef SITL_TLP_LINK_HPP
#define SITL_TLP_LINK_HPP

#include "asp_tlp64.hpp"
#include "tlp_ring.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

namespace abstractx::target::sitl {

struct TlpLinkCounters {
    uint64_t tlps_in{0u};
    uint64_t tlps_out{0u};
    uint64_t dropped_ring_full{0u};
    uint64_t unexpected_type{0u};
    uint64_t disconnects{0u};
    uint64_t refused_connections{0u};
};

class SitlTlpLink {
public:
    static constexpr size_t kTlpBytes = 64u;
    static_assert(sizeof(Tlp64) == kTlpBytes, "a TLP on the link is exactly 64 bytes");

    // Rings are owned by the caller and outlive the link.
    SitlTlpLink(SpscTlpRing<64u>& stream_ring, SpscTlpRing<64u>& completion_ring) noexcept
        : stream_ring_{stream_ring}, completion_ring_{completion_ring} {}

    SitlTlpLink(const SitlTlpLink&) = delete;
    SitlTlpLink& operator=(const SitlTlpLink&) = delete;
    ~SitlTlpLink() { close_all(); }

    // Listen on bind_addr:port. Port 0 picks a free port (see port()).
    [[nodiscard]] bool listen(const char* bind_addr, uint16_t port) noexcept {
        listen_fd_ = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
        if (listen_fd_ < 0) {
            return false;
        }
        const int one = 1;
        (void)::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(port);
        if (::inet_pton(AF_INET, bind_addr, &addr.sin_addr) != 1 ||
            ::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0 ||
            ::listen(listen_fd_, 1) != 0) {
            close_fd(listen_fd_);
            return false;
        }
        socklen_t len = sizeof(addr);
        if (::getsockname(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &len) == 0) {
            port_ = ntohs(addr.sin_port);
        }
        return true;
    }

    [[nodiscard]] uint16_t port() const noexcept { return port_; }
    [[nodiscard]] bool connected() const noexcept { return conn_fd_ >= 0; }
    [[nodiscard]] const TlpLinkCounters& counters() const noexcept { return counters_; }

    // True when Nagle is off on the open connection, read back from the socket.
    [[nodiscard]] bool nodelay() const noexcept {
        int v = 0;
        socklen_t l = sizeof(v);
        return conn_fd_ >= 0 && ::getsockopt(conn_fd_, IPPROTO_TCP, TCP_NODELAY, &v, &l) == 0 && v != 0;
    }

    // Wait up to timeout_ms for a connection or data, then route every whole TLP received.
    // This is the SITL host's wait (the "ISR" of this target); it is never called from a coroutine.
    // Returns the number of TLPs routed.
    size_t poll(int timeout_ms) noexcept {
        pollfd fds[2]{};
        nfds_t n = 0u;
        if (listen_fd_ >= 0) {
            fds[n++] = pollfd{listen_fd_, POLLIN, 0};
        }
        if (conn_fd_ >= 0) {
            fds[n++] = pollfd{conn_fd_, POLLIN, 0};
        }
        if (n == 0u || ::poll(fds, n, timeout_ms) <= 0) {
            return 0u;
        }
        for (nfds_t i = 0u; i < n; ++i) {
            if (fds[i].fd == listen_fd_ && (fds[i].revents & POLLIN) != 0) {
                accept_one();
            }
        }
        return conn_fd_ >= 0 ? receive() : 0u;
    }

    // Send one TLP: its 64 bytes as they are. The link adds and checks nothing.
    [[nodiscard]] bool send(const Tlp64& tlp) noexcept {
        if (conn_fd_ < 0) {
            return false;
        }
        const auto* frame = reinterpret_cast<const uint8_t*>(&tlp);
        size_t sent = 0u;
        while (sent < kTlpBytes) {
            const ssize_t r = ::send(conn_fd_, frame + sent, kTlpBytes - sent, MSG_NOSIGNAL);
            if (r > 0) {
                sent += static_cast<size_t>(r);
            } else if (r < 0 && errno == EINTR) {
                continue;
            } else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                pollfd w{conn_fd_, POLLOUT, 0};
                (void)::poll(&w, 1u, 100);
            } else {
                drop_connection();
                return false;
            }
        }
        ++counters_.tlps_out;
        return true;
    }

    // Send every TLP waiting in an outbound ring (e.g. TlpChannel's requests).
    size_t drain(SpscTlpRing<64u>& outbound) noexcept {
        size_t n = 0u;
        while (auto tlp = outbound.pop()) {
            if (!send(*tlp)) {
                break;
            }
            ++n;
        }
        return n;
    }

private:
    SpscTlpRing<64u>& stream_ring_;
    SpscTlpRing<64u>& completion_ring_;
    int listen_fd_{-1};
    int conn_fd_{-1};
    uint16_t port_{0u};
    uint8_t rx_[kTlpBytes]{};
    size_t rx_len_{0u};
    TlpLinkCounters counters_{};

    void accept_one() noexcept {
        const int fd = ::accept4(listen_fd_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
        if (fd < 0) {
            return;
        }
        if (conn_fd_ >= 0) {
            ++counters_.refused_connections;
            ::close(fd);
            return;
        }
        const int one = 1;
        (void)::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        conn_fd_ = fd;
        rx_len_ = 0u;
    }

    size_t receive() noexcept {
        size_t routed = 0u;
        for (;;) {
            const ssize_t r = ::recv(conn_fd_, rx_ + rx_len_, kTlpBytes - rx_len_, 0);
            if (r > 0) {
                rx_len_ += static_cast<size_t>(r);
                if (rx_len_ == kTlpBytes) {
                    route();
                    rx_len_ = 0u;
                    ++routed;
                }
            } else if (r < 0 && errno == EINTR) {
                continue;
            } else if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                return routed;
            } else {
                drop_connection();
                return routed;
            }
        }
    }

    void route() noexcept {
        Tlp64 tlp{};
        std::memcpy(&tlp, rx_, kTlpBytes);
        ++counters_.tlps_in;
        SpscTlpRing<64u>* ring = nullptr;
        switch (tlp.wire.type) {
        case ASP_TLP_TYPE_DMA_STREAM: ring = &stream_ring_; break;
        case ASP_TLP_TYPE_CPL_D:
        case ASP_TLP_TYPE_CPL:        ring = &completion_ring_; break;
        default:                      ++counters_.unexpected_type; return;
        }
        if (!ring->push(tlp)) {
            ++counters_.dropped_ring_full;
        }
    }

    void drop_connection() noexcept {
        close_fd(conn_fd_);
        rx_len_ = 0u; // a partly received TLP is discarded
        ++counters_.disconnects;
    }

    static void close_fd(int& fd) noexcept {
        if (fd >= 0) {
            ::close(fd);
            fd = -1;
        }
    }

    void close_all() noexcept {
        close_fd(conn_fd_);
        close_fd(listen_fd_);
    }
};

} // namespace abstractx::target::sitl

#endif // SITL_TLP_LINK_HPP
