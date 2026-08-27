/**
 * \file lightspeed/telemetry/serial_link.hpp
 *
 * Bench-tuning only: streams the telemetry bus over the existing USB stdout
 * as one CSV line per cycle (a header line, then one data line per sample),
 * so a laptop-side script can tail the terminal and live-plot values. Not
 * part of competition operation -- see main.cpp for where this is (not)
 * started.
 */

#pragma once

#include <cstdint>
#include <optional>

#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::telemetry {

class SerialLink {
public:
    explicit SerialLink(const TelemetryBus& bus);
    ~SerialLink();

    SerialLink(const SerialLink&) = delete;
    SerialLink& operator=(const SerialLink&) = delete;

    // Starts the background streaming task. No-op if already started.
    void start();

private:
    void linkLoop();

    const TelemetryBus& bus_;
    bool headerPrinted_ = false;
    std::uint8_t headerColumnCount_ = 0;

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::telemetry
