/**
 * \file lightspeed/telemetry/telemetry_constants.hpp
 *
 * Single source of truth for every telemetry-module rate/size knob: SD
 * logger sample and flush cadence, dashboard redraw rate, and the serial
 * link's bench-tuning stream rate.
 */

#pragma once

#include <cstdint>

namespace lightspeed::telemetry {

// -- SD logger --
// Row sample rate: how often a bus snapshot is formatted into a CSV row and
// appended to the RAM buffer. This is the log's temporal resolution.
inline constexpr std::uint32_t kSdLoggerSampleIntervalMs = 40;  // ~25Hz

// Rows accumulated in RAM before a single buffered fwrite+fflush to the SD
// card -- decouples "how often we sample" from "how often we touch the
// card", which is what actually bounds SD wear and blocking-call frequency.
inline constexpr std::uint16_t kSdLoggerRowsPerFlush = 10;  // flush roughly every 400ms

// Reserved once at construction (see SdLogger) so appending rows between
// flushes never reallocates -- sized generously for kMaxChannels columns
// x kSdLoggerRowsPerFlush rows of formatted text.
inline constexpr std::size_t kSdLoggerBufferReserveBytes = 8192;

inline constexpr const char* kSdLoggerDirectory = "/usd";
// 8.3-safe basename (FatFS on the V5 may not support long file names):
// "lslog000.csv" .. "lslog999.csv". A run's file is the first index that
// doesn't already exist, so re-running (even across power cycles) never
// overwrites a previous match's log.
inline constexpr const char* kSdLoggerFilenameFormat = "%s/lslog%03u.csv";
inline constexpr unsigned int kSdLoggerMaxFileIndex = 1000;

// -- Dashboard --
inline constexpr std::uint32_t kDashboardLoopPeriodMs = 125;  // ~8Hz

// -- Serial link (bench-tuning only, see serial_link.hpp) --
inline constexpr std::uint32_t kSerialLinkLoopPeriodMs = 100;  // ~10Hz

}  // namespace lightspeed::telemetry
