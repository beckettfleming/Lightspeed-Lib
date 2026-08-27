/**
 * \file lightspeed/telemetry/sd_logger.hpp
 *
 * Buffered CSV logger: a dedicated low-priority background task polls the
 * telemetry bus at a fixed sample rate, formats each snapshot as one CSV
 * row into a RAM buffer, and periodically flushes several rows at once to
 * the SD card. Never writes synchronously from a timing-sensitive loop --
 * drivetrain/odometry code only ever calls TelemetryBus::record(), which is
 * a plain in-memory write.
 *
 * A new log file is opened on the disabled->enabled competition-state
 * transition (not at program boot), so re-running a match doesn't overwrite
 * the previous one -- see the edge detection in loggerLoop().
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

    // Closes any currently-open file, then opens a fresh one at the first
    // unused index (see kSdLoggerFilenameFormat) -- safe to call even if no
    // SD card is present (file_ just stays null and logging is skipped).
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
