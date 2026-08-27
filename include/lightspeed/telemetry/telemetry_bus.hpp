/**
 * \file lightspeed/telemetry/telemetry_bus.hpp
 *
 * Central named-channel registry: any layer calls record(name, value) on
 * its own update cycle to publish its latest reading. This is a passive
 * shared snapshot with many producers -- it does not own a polling loop or
 * decide how often anything gets sampled; SdLogger and Dashboard each poll
 * getSnapshot() at their own independent rate.
 *
 * Channel names must be stable-lifetime strings (string literals, as used
 * everywhere else in this project -- see FlagRegistry, Routine::name) --
 * the bus stores the pointer, not a copy.
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

    // Publishes the latest value for a named channel, registering it (in
    // call order, stable thereafter) on first use. Safe to call from any
    // task, every cycle -- a plain array scan under a short-lived mutex, no
    // allocation. A plain `float` argument converts to double implicitly.
    void record(const char* name, double value);
    void record(const char* name, bool value);
    void record(const char* name, std::int32_t value);
    void record(const char* name, const char* value);

    static constexpr std::uint8_t kMaxChannels = 32;

    struct Snapshot {
        std::array<ChannelSnapshot, kMaxChannels> channels{};
        std::uint8_t count = 0;
    };

    // Copies every currently-registered channel's latest value. Cheap
    // (fixed-size array copy, no allocation) and safe to call from any task
    // at its own rate.
    [[nodiscard]] Snapshot getSnapshot() const;

private:
    TelemetryBus() = default;

    // Caller must hold mutex_. Returns the channel's index, registering a
    // new one (appended, so ordering stays stable) if `name` hasn't been
    // seen before, or -1 if the registry is full.
    [[nodiscard]] int findOrCreateIndexLocked(const char* name);

    std::array<ChannelSnapshot, kMaxChannels> channels_{};
    std::uint8_t count_ = 0;
    mutable pros::Mutex mutex_;
};

}  // namespace lightspeed::telemetry
