/**
 * \file lightspeed/telemetry/telemetry_bus.hpp
 *
 * Central named-channel registry: any layer calls record(name, value) on its
 * own update cycle. Passive -- it owns no polling loop and decides nothing
 * about sample rates; consumers poll getSnapshot() at their own rate.
 *
 * Channel names MUST be stable-lifetime strings (string literals) -- the bus
 * stores the pointer, not a copy.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/telemetry/
 */

#pragma once

#include <array>
#include <cstdint>

#include "pros/rtos.hpp"

namespace lightspeed::telemetry {

enum class ChannelType : std::uint8_t { number, integer, boolean, text };

struct ChannelSnapshot {
    const char* name = nullptr;
    ChannelType type = ChannelType::number;
    double number = 0.0;
    std::int32_t integer = 0;
    bool boolean = false;
    const char* text = "";
};

class TelemetryBus {
public:
    static TelemetryBus& instance();

    TelemetryBus(const TelemetryBus&) = delete;
    TelemetryBus& operator=(const TelemetryBus&) = delete;

    // Registers the channel on first use, in call order, stable thereafter
    // (which is what keeps the CSV header valid for a whole run). Safe from
    // any task every cycle -- an array scan under a short mutex, no
    // allocation.
    void record(const char* name, double value);
    void record(const char* name, bool value);
    void record(const char* name, std::int32_t value);
    void record(const char* name, const char* value);

    static constexpr std::uint8_t kMaxChannels = 32;

    struct Snapshot {
        std::array<ChannelSnapshot, kMaxChannels> channels{};
        std::uint8_t count = 0;
    };

    // Fixed-size array copy, no allocation. Safe from any task.
    [[nodiscard]] Snapshot getSnapshot() const;

private:
    TelemetryBus() = default;

    // Caller must hold mutex_. Appends a new channel if unseen, keeping
    // ordering stable; -1 if the registry is full.
    [[nodiscard]] int findOrCreateIndexLocked(const char* name);

    std::array<ChannelSnapshot, kMaxChannels> channels_{};
    std::uint8_t count_ = 0;
    mutable pros::Mutex mutex_;
};

}  // namespace lightspeed::telemetry
