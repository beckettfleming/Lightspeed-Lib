/**
 * \file lightspeed/telemetry/sd_logger.hpp
 *
 * Buffered CSV logger: a low-priority background task polls the telemetry
 * bus at a fixed rate, formats each snapshot into a RAM buffer, and flushes
 * several rows at once. Never writes synchronously from a timing-sensitive
 * loop -- those only ever call TelemetryBus::record().
 *
 * A new log file opens on the disabled->enabled competition-state
 * transition, not at boot, so one file is produced per match.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/telemetry/
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>

#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::telemetry {

class SdLogger {
public:
    explicit SdLogger(const TelemetryBus& bus);
    ~SdLogger();

    // Owns a background task referencing `this` -- not safe to copy or move.
    SdLogger(const SdLogger&) = delete;
    SdLogger& operator=(const SdLogger&) = delete;

    // Starts the background logging task. No-op if already started.
    void start();

private:
    void loggerLoop();

    // Opens the first unused index. Safe with no SD card present -- file_
    // stays null and logging is skipped.
    void startNewLogFile();

    void appendRow(const TelemetryBus::Snapshot& snapshot);
    void flushIfDue(bool force);

    const TelemetryBus& bus_;

    std::FILE* file_ = nullptr;
    bool headerWritten_ = false;
    std::uint8_t headerColumnCount_ = 0;

    std::string rowBuffer_;
    std::uint16_t bufferedRowCount_ = 0;

    // Edge-detects the disabled->enabled transition; starts true so a boot
    // that begins already-enabled (e.g. untethered bench testing) still
    // opens a file on its first loggerLoop() cycle.
    bool previouslyDisabled_ = true;

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::telemetry
