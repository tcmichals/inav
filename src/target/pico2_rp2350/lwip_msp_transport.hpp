/*
 * Copyright (C) 2026 Tim Michals
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `tcmichals/inav` - lwIP Raw API TCP Transport for Pico 2 W (RP2350 + CYW43439)
 *
 * Satisfies MspTransport concept using lwIP raw (NO_SYS=1) callbacks.
 * Zero copies, zero threads, zero malloc. Callbacks fire directly from cyw43_arch_poll().
 *
 * NOTE: This file only compiles under the Pico SDK CMake environment.
 */

#ifndef LWIP_MSP_TRANSPORT_HPP
#define LWIP_MSP_TRANSPORT_HPP

#if defined(PICO_BOARD)

#include "msp_transport.hpp"

// lwIP raw API headers
#include "lwip/tcp.h"
#include "lwip/pbuf.h"

#include <functional>
#include <cstring>

namespace abstractx::msp {

class LwipMspTransport {
public:
    using OnBytesReceivedCb = std::function<void(std::span<const uint8_t>)>;

    LwipMspTransport() noexcept = default;

    void set_on_bytes_received(OnBytesReceivedCb cb) noexcept {
        on_bytes_received_ = std::move(cb);
    }

    bool start(uint16_t port) noexcept {
        listen_pcb_ = tcp_new();
        if (!listen_pcb_) return false;

        err_t err = tcp_bind(listen_pcb_, IP_ADDR_ANY, port);
        if (err != ERR_OK) {
            tcp_close(listen_pcb_);
            listen_pcb_ = nullptr;
            return false;
        }

        listen_pcb_ = tcp_listen(listen_pcb_);
        if (!listen_pcb_) return false;

        // Pass `this` as callback arg
        tcp_arg(listen_pcb_, this);
        tcp_accept(listen_pcb_, on_accept_cb);

        running_ = true;
        return true;
    }

    void stop() noexcept {
        running_ = false;
        if (client_pcb_) {
            tcp_recv(client_pcb_, nullptr);
            tcp_err(client_pcb_, nullptr);
            tcp_close(client_pcb_);
            client_pcb_ = nullptr;
        }
        if (listen_pcb_) {
            tcp_close(listen_pcb_);
            listen_pcb_ = nullptr;
        }
        client_connected_ = false;
    }

    // No-op on Pico 2 W — lwIP callbacks are driven by cyw43_arch_poll()
    // in the main loop. The callbacks have already fired by the time this is called.
    void poll() noexcept {
        // lwIP raw API is event-driven, not polled.
        // cyw43_arch_poll() in Pico main loop drives lwIP timers and rx.
    }

    bool send(std::span<const uint8_t> data) noexcept {
        if (!client_connected_ || !client_pcb_) return false;

        err_t err = tcp_write(client_pcb_, data.data(),
                              static_cast<u16_t>(data.size()),
                              TCP_WRITE_FLAG_COPY);
        if (err != ERR_OK) return false;

        tcp_output(client_pcb_);
        return true;
    }

    bool has_client() const noexcept {
        return client_connected_;
    }

private:
    struct tcp_pcb* listen_pcb_{nullptr};
    struct tcp_pcb* client_pcb_{nullptr};
    bool running_{false};
    bool client_connected_{false};
    OnBytesReceivedCb on_bytes_received_;

    // ---- lwIP Raw API Static Callbacks ----
    // These are static C-style callbacks as required by lwIP. The `arg` pointer
    // carries `this` to route back into the C++ object.

    static err_t on_accept_cb(void* arg, struct tcp_pcb* newpcb, err_t err) {
        if (err != ERR_OK || !newpcb) return ERR_VAL;

        auto* self = static_cast<LwipMspTransport*>(arg);

        // Close existing client if any (single-client server)
        if (self->client_pcb_) {
            tcp_recv(self->client_pcb_, nullptr);
            tcp_err(self->client_pcb_, nullptr);
            tcp_close(self->client_pcb_);
        }

        self->client_pcb_ = newpcb;
        self->client_connected_ = true;

        tcp_arg(newpcb, self);
        tcp_recv(newpcb, on_recv_cb);
        tcp_err(newpcb, on_err_cb);

        return ERR_OK;
    }

    static err_t on_recv_cb(void* arg, struct tcp_pcb* pcb, struct pbuf* p, err_t err) {
        auto* self = static_cast<LwipMspTransport*>(arg);

        if (!p || err != ERR_OK) {
            // Client disconnected
            if (p) pbuf_free(p);
            self->client_connected_ = false;
            self->client_pcb_ = nullptr;
            tcp_close(pcb);
            return ERR_OK;
        }

        // Walk pbuf chain — lwIP may split data across chained pbufs
        for (struct pbuf* q = p; q != nullptr; q = q->next) {
            if (self->on_bytes_received_ && q->len > 0) {
                self->on_bytes_received_(
                    std::span<const uint8_t>(
                        static_cast<const uint8_t*>(q->payload),
                        q->len));
            }
        }

        // Acknowledge received bytes to lwIP TCP window
        tcp_recved(pcb, p->tot_len);
        pbuf_free(p);
        return ERR_OK;
    }

    static void on_err_cb(void* arg, err_t /*err*/) {
        auto* self = static_cast<LwipMspTransport*>(arg);
        self->client_connected_ = false;
        self->client_pcb_ = nullptr;
        // pcb is already freed by lwIP when on_err fires
    }
};

// Static assertion: LwipMspTransport satisfies MspTransport concept
static_assert(MspTransport<LwipMspTransport>,
    "LwipMspTransport must satisfy MspTransport concept");

} // namespace abstractx::msp

#endif // PICO_BOARD

#endif // LWIP_MSP_TRANSPORT_HPP
