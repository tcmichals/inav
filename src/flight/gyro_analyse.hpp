/*
 * Copyright (C) 2026 Tim Michals
 * Copyright (C) 2018-2026 INAV Contributors (Konstantin Sharlaimov, Pawel Spychalski, et al.)
 * Copyright (C) 2018-2026 Betaflight Contributors (ctzsnooze, Rav, DieHertz, eTracer, et al.)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 * `inav-abstractx` - Production Dynamic Gyro Notch & Spectral Frequency Analyzer
 *
 * Exact C++20 Reference Port of Upstream INAV C Source:
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/gyroanalyse.c`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/flight/gyroanalyse.h`
 *   - `/home/tcmichals/ssdData/projects/home/flightcode/inav/src/main/sensors/gyro.c`
 *
 * Features:
 * 1. 64-Point Sliding Window Real FFT (RFFT) with Downsampling Denominator.
 * 2. 4-Stage State Machine (Windowing, FFT, Peak Magnitude Detection, Filter Update).
 * 3. Hanning Windowing ($w_n = 0.5 - 0.5 \cos(2\pi n / (N-1))$).
 * 4. Sub-Bin Parabolic Interpolation ($y(x) \to$ exact continuous peak frequency).
 * 5. N-Peak Harmonic Noise Tracking with Ascending Bin Sorting.
 * 6. 25 Hz PT1 Smoothing on Detected Notch Frequencies.
 * 7. Dynamic Biquad Notch Cascades (Primary + Secondary Harmonic Notches).
 *
 * Standards: MISRA C++:2023, NASA/JPL Power of 10 (Zero dynamic allocation, [[nodiscard]], const noexcept)
 */

#ifndef FLIGHT_GYRO_ANALYSE_HPP
#define FLIGHT_GYRO_ANALYSE_HPP

#include <cstdint>
#include <cmath>
#include <array>
#include <algorithm>
#include "filter.hpp"

namespace abstractx::flight {

inline constexpr size_t DYN_NOTCH_WINDOW_SIZE      = 64;
inline constexpr size_t DYN_NOTCH_BIN_COUNT        = DYN_NOTCH_WINDOW_SIZE / 2; // 32 frequency bins
inline constexpr size_t DYN_NOTCH_PEAK_COUNT       = 3;  // Track up to 3 harmonic noise peaks
inline constexpr float  DYN_NOTCH_SMOOTH_FREQ_HZ   = 25.0f; // 25 Hz PT1 frequency smoothing
inline constexpr size_t DYN_NOTCH_AXIS_COUNT       = 3;  // Roll, Pitch, Yaw
inline constexpr uint8_t FFT_SAMPLING_DENOMINATOR  = 2;  // Downsample by 2 (e.g. 1kHz -> 500Hz Nyquist)

struct Peak {
    int bin{0};
    float value{0.0f};
};

enum class AnalysisStep : uint8_t {
    StepWindowing = 0,
    StepRfft,
    StepMagnitudeAndPeaks,
    StepUpdateFilters,
    StepCount
};

// -----------------------------------------------------------------------------
// Dynamic Gyro Notch Spectral Analyzer Class (gyroAnalyseState_t)
// -----------------------------------------------------------------------------
class GyroSpectralAnalyzer {
public:
    GyroSpectralAnalyzer() noexcept {
        init(100, 1000); // Default: 100Hz min frequency, 1000us looptime (1kHz)
    }

    void init(uint16_t min_frequency_hz, uint32_t target_looptime_us) noexcept {
        min_frequency_hz_ = min_frequency_hz;
        target_looptime_us_ = target_looptime_us;

        fft_sampling_rate_hz_ = static_cast<uint16_t>(1000000.0f / (target_looptime_us * FFT_SAMPLING_DENOMINATOR));
        max_frequency_hz_ = fft_sampling_rate_hz_ / 2; // Nyquist limit
        fft_resolution_ = static_cast<float>(max_frequency_hz_) / DYN_NOTCH_BIN_COUNT;

        const int res_int = std::max(1, static_cast<int>(std::lrint(fft_resolution_)));
        fft_start_bin_ = static_cast<uint8_t>(min_frequency_hz_ / res_int);

        // Precompute Hanning Window (0.5 - 0.5 * cos(2 * pi * i / (N - 1)))
        constexpr float TWO_PI = 6.283185307179586f;
        for (size_t i = 0; i < DYN_NOTCH_WINDOW_SIZE; ++i) {
            hanning_window_[i] = 0.5f - 0.5f * std::cos(TWO_PI * i / (DYN_NOTCH_WINDOW_SIZE - 1));
        }

        // Initialize 25Hz PT1 Frequency Smoothing Filters
        const uint32_t filter_update_us = target_looptime_us * static_cast<uint32_t>(AnalysisStep::StepCount) * DYN_NOTCH_AXIS_COUNT;
        const float filter_update_dt_s = static_cast<float>(filter_update_us) * 1e-6f;

        for (size_t axis = 0; axis < DYN_NOTCH_AXIS_COUNT; ++axis) {
            for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
                center_frequency_[axis][p] = static_cast<float>(max_frequency_hz_);
                detected_frequency_filter_[axis][p].reset(center_frequency_[axis][p]);
                detected_frequency_filter_[axis][p].set_cutoff(DYN_NOTCH_SMOOTH_FREQ_HZ, filter_update_dt_s);
            }
        }

        circular_buffer_idx_ = 0;
        update_step_ = AnalysisStep::StepWindowing;
        update_axis_ = 0;
        filter_update_execute_ = false;
        filter_update_axis_ = 0;
    }

    // -------------------------------------------------------------------------
    // 1. Push Raw Gyro Sample for Specified Axis (gyroDataAnalysePush)
    // -------------------------------------------------------------------------
    void push(size_t axis, float sample) noexcept {
        if (axis < DYN_NOTCH_AXIS_COUNT) {
            current_sample_[axis] = sample;
        }
    }

    // -------------------------------------------------------------------------
    // 2. Advance Spectral Analysis State Machine (gyroDataAnalyse)
    // -------------------------------------------------------------------------
    void update() noexcept {
        filter_update_execute_ = false;

        if (sampling_index_ == 0) {
            for (size_t axis = 0; axis < DYN_NOTCH_AXIS_COUNT; ++axis) {
                downsampled_gyro_data_[axis][circular_buffer_idx_] = current_sample_[axis];
            }
            circular_buffer_idx_ = (circular_buffer_idx_ + 1) % DYN_NOTCH_WINDOW_SIZE;
        }

        sampling_index_ = (sampling_index_ + 1) % FFT_SAMPLING_DENOMINATOR;

        // Run 4-stage update cycle
        run_state_machine_step();
    }

    [[nodiscard]] bool has_filter_update() const noexcept { return filter_update_execute_; }
    [[nodiscard]] size_t filter_update_axis() const noexcept { return filter_update_axis_; }
    [[nodiscard]] float center_frequency(size_t axis, size_t peak_idx) const noexcept {
        if (axis < DYN_NOTCH_AXIS_COUNT && peak_idx < DYN_NOTCH_PEAK_COUNT) {
            return center_frequency_[axis][peak_idx];
        }
        return 0.0f;
    }

private:
    // Sub-bin parabolic interpolation over peak bin and shoulder bins (y0, y1, y2)
    [[nodiscard]] float compute_parabola_mean(size_t peak_bin) const noexcept {
        float precise_bin = static_cast<float>(peak_bin);
        if (peak_bin > 0 && peak_bin < DYN_NOTCH_BIN_COUNT - 1) {
            const float y0 = fft_magnitude_[peak_bin - 1];
            const float y1 = fft_magnitude_[peak_bin];
            const float y2 = fft_magnitude_[peak_bin + 1];

            const float denom = 2.0f * (y0 - 2.0f * y1 + y2);
            if (std::abs(denom) > 1e-6f) {
                precise_bin += std::clamp((y0 - y2) / denom, -0.5f, 0.5f);
            }
        }
        return precise_bin;
    }

    // 4-Stage Analysis Cycle (gyroDataAnalyseUpdate)
    void run_state_machine_step() noexcept {
        switch (update_step_) {
            case AnalysisStep::StepWindowing: {
                // Apply Hanning Window to gyro samples
                const size_t start_idx = circular_buffer_idx_;
                for (size_t i = 0; i < DYN_NOTCH_WINDOW_SIZE; ++i) {
                    const size_t buf_idx = (start_idx + i) % DYN_NOTCH_WINDOW_SIZE;
                    fft_real_[i] = downsampled_gyro_data_[update_axis_][buf_idx] * hanning_window_[i];
                    fft_imag_[i] = 0.0f;
                }
                update_step_ = AnalysisStep::StepRfft;
                break;
            }

            case AnalysisStep::StepRfft: {
                // Compute 64-Point In-Place Real FFT
                compute_fft_64(fft_real_.data(), fft_imag_.data());
                update_step_ = AnalysisStep::StepMagnitudeAndPeaks;
                break;
            }

            case AnalysisStep::StepMagnitudeAndPeaks: {
                // Compute complex magnitudes: |X[k]| = sqrt(re^2 + im^2)
                for (size_t bin = 0; bin < DYN_NOTCH_BIN_COUNT; ++bin) {
                    fft_magnitude_[bin] = std::sqrt(fft_real_[bin] * fft_real_[bin] + fft_imag_[bin] * fft_imag_[bin]);
                }

                // Reset peak list
                for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
                    peaks_[p] = Peak{0, 0.0f};
                }

                // Find local maxima peaks
                const size_t start_bin = std::max<size_t>(1, fft_start_bin_);
                for (size_t bin = start_bin; bin < DYN_NOTCH_BIN_COUNT - 1; ++bin) {
                    if (fft_magnitude_[bin] > fft_magnitude_[bin - 1] &&
                        fft_magnitude_[bin] > fft_magnitude_[bin + 1]) {

                        // Insert into top N peaks
                        for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
                            if (fft_magnitude_[bin] > peaks_[p].value) {
                                for (size_t k = DYN_NOTCH_PEAK_COUNT - 1; k > p; --k) {
                                    peaks_[k] = peaks_[k - 1];
                                }
                                peaks_[p].bin = static_cast<int>(bin);
                                peaks_[p].value = fft_magnitude_[bin];
                                break;
                            }
                        }
                        bin++; // Skip adjacent bin
                    }
                }

                // Sort peaks in ascending frequency order
                for (size_t p = DYN_NOTCH_PEAK_COUNT - 1; p > 0; --p) {
                    for (size_t k = 0; k < p; ++k) {
                        if (peaks_[k].bin > peaks_[k + 1].bin && peaks_[k + 1].bin != 0) {
                            std::swap(peaks_[k], peaks_[k + 1]);
                        }
                    }
                }

                update_step_ = AnalysisStep::StepUpdateFilters;
                break;
            }

            case AnalysisStep::StepUpdateFilters: {
                // Update continuous smoothed center frequencies
                for (size_t i = 0; i < DYN_NOTCH_PEAK_COUNT; ++i) {
                    if (peaks_[i].bin > 0) {
                        const size_t clamped_bin = std::clamp<size_t>(peaks_[i].bin, fft_start_bin_, DYN_NOTCH_BIN_COUNT - 1);
                        const float freq = compute_parabola_mean(clamped_bin) * fft_resolution_;
                        center_frequency_[update_axis_][i] = detected_frequency_filter_[update_axis_][i].update(freq);
                    } else {
                        center_frequency_[update_axis_][i] = 0.0f;
                    }
                }

                filter_update_execute_ = true;
                filter_update_axis_ = update_axis_;

                // Advance to next axis (Roll -> Pitch -> Yaw)
                update_axis_ = (update_axis_ + 1) % DYN_NOTCH_AXIS_COUNT;
                update_step_ = AnalysisStep::StepWindowing;
                break;
            }

            default:
                update_step_ = AnalysisStep::StepWindowing;
                break;
        }
    }

    // Fast Radix-2 Cooley-Tukey in-place 64-point FFT
    static void compute_fft_64(float* re, float* im) noexcept {
        constexpr size_t N = 64;

        // Bit-reversal permutation
        size_t j = 0;
        for (size_t i = 0; i < N - 1; ++i) {
            if (i < j) {
                std::swap(re[i], re[j]);
                std::swap(im[i], im[j]);
            }
            size_t k = N >> 1;
            while (k <= j) {
                j -= k;
                k >>= 1;
            }
            j += k;
        }

        // Cooley-Tukey butterfly stages
        for (size_t len = 2; len <= N; len <<= 1) {
            const float angle = -6.283185307179586f / len;
            const float w_len_re = std::cos(angle);
            const float w_len_im = std::sin(angle);

            for (size_t i = 0; i < N; i += len) {
                float w_re = 1.0f;
                float w_im = 0.0f;
                const size_t half_len = len >> 1;

                for (size_t u = 0; u < half_len; ++u) {
                    const size_t idx_u = i + u;
                    const size_t idx_v = idx_u + half_len;

                    const float u_re = re[idx_u];
                    const float u_im = im[idx_u];
                    const float v_re = re[idx_v] * w_re - im[idx_v] * w_im;
                    const float v_im = re[idx_v] * w_im + im[idx_v] * w_re;

                    re[idx_u] = u_re + v_re;
                    im[idx_u] = u_im + v_im;
                    re[idx_v] = u_re - v_re;
                    im[idx_v] = u_im - v_im;

                    const float next_w_re = w_re * w_len_re - w_im * w_len_im;
                    w_im = w_re * w_len_im + w_im * w_len_re;
                    w_re = next_w_re;
                }
            }
        }
    }

    uint16_t min_frequency_hz_{100};
    uint16_t max_frequency_hz_{250};
    uint16_t fft_sampling_rate_hz_{500};
    uint8_t fft_start_bin_{2};
    float fft_resolution_{7.8125f};
    uint32_t target_looptime_us_{1000};

    uint8_t sampling_index_{0};
    size_t circular_buffer_idx_{0};

    std::array<float, DYN_NOTCH_AXIS_COUNT> current_sample_{{0.0f, 0.0f, 0.0f}};
    std::array<std::array<float, DYN_NOTCH_WINDOW_SIZE>, DYN_NOTCH_AXIS_COUNT> downsampled_gyro_data_{};

    std::array<float, DYN_NOTCH_WINDOW_SIZE> hanning_window_{};
    std::array<float, DYN_NOTCH_WINDOW_SIZE> fft_real_{};
    std::array<float, DYN_NOTCH_WINDOW_SIZE> fft_imag_{};
    std::array<float, DYN_NOTCH_BIN_COUNT> fft_magnitude_{};

    std::array<Peak, DYN_NOTCH_PEAK_COUNT> peaks_{};
    std::array<std::array<float, DYN_NOTCH_PEAK_COUNT>, DYN_NOTCH_AXIS_COUNT> center_frequency_{};
    std::array<std::array<Pt1Filter, DYN_NOTCH_PEAK_COUNT>, DYN_NOTCH_AXIS_COUNT> detected_frequency_filter_{};

    AnalysisStep update_step_{AnalysisStep::StepWindowing};
    size_t update_axis_{0};
    bool filter_update_execute_{false};
    size_t filter_update_axis_{0};
};

// -----------------------------------------------------------------------------
// Dynamic Gyro Notch Tracking Filter Cascades (dynamicGyroNotchFiltersApply)
// -----------------------------------------------------------------------------
class DynamicGyroNotchBank {
public:
    DynamicGyroNotchBank() noexcept = default;

    void init(float looptime_dt_s) noexcept {
        looptime_dt_s_ = looptime_dt_s;
        enabled_ = true;
        for (size_t axis = 0; axis < DYN_NOTCH_AXIS_COUNT; ++axis) {
            for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
                notch_filters_[axis][p].reset();
                notch_filters_[axis][p].set_notch(200.0f, 2.0f, looptime_dt_s_);
            }
        }
    }

    void update_frequencies(size_t axis, const std::array<float, DYN_NOTCH_PEAK_COUNT>& freqs) noexcept {
        if (axis < DYN_NOTCH_AXIS_COUNT) {
            for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
                if (freqs[p] > 50.0f) {
                    notch_filters_[axis][p].set_notch(freqs[p], 2.5f, looptime_dt_s_);
                }
            }
        }
    }

    [[nodiscard]] float apply(size_t axis, float input) noexcept {
        if (!enabled_ || axis >= DYN_NOTCH_AXIS_COUNT) {
            return input;
        }
        float out = input;
        for (size_t p = 0; p < DYN_NOTCH_PEAK_COUNT; ++p) {
            out = notch_filters_[axis][p].update(out);
        }
        return out;
    }

    void set_enabled(bool enabled) noexcept { enabled_ = enabled; }
    [[nodiscard]] bool is_enabled() const noexcept { return enabled_; }

private:
    float looptime_dt_s_{0.001f};
    bool enabled_{true};
    std::array<std::array<BiquadFilter, DYN_NOTCH_PEAK_COUNT>, DYN_NOTCH_AXIS_COUNT> notch_filters_{};
};

// -----------------------------------------------------------------------------
// Direct 1:1 C-API Compatibility Wrappers (Matching upstream INAV gyroanalyse.h)
// -----------------------------------------------------------------------------
using gyroAnalyseState_t = GyroSpectralAnalyzer;

inline void gyroDataAnalyseStateInit(gyroAnalyseState_t* state, uint16_t min_freq, uint32_t looptime_us) noexcept {
    if (state != nullptr) {
        state->init(min_freq, looptime_us);
    }
}

inline void gyroDataAnalysePush(gyroAnalyseState_t* state, int axis, float sample) noexcept {
    if (state != nullptr) {
        state->push(static_cast<size_t>(axis), sample);
    }
}

inline void gyroDataAnalyse(gyroAnalyseState_t* state) noexcept {
    if (state != nullptr) {
        state->update();
    }
}

} // namespace abstractx::flight

#endif // FLIGHT_GYRO_ANALYSE_HPP

