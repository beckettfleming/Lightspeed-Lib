/**
 * \file lightspeed/telemetry/serial_link.hpp
 *
 * Bench-tuning only: streams the telemetry bus over USB stdout as CSV (a
 * header line, then one data line per sample) so a laptop-side script can
 * tail the terminal and live-plot values. Never started during normal
 * competition operation -- its only entry point is diagnostic mode.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/diagnostics/
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
