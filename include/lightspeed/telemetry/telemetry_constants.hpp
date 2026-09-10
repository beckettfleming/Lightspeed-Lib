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
// The log's temporal resolution.
inline constexpr std::uint32_t kSdLoggerSampleIntervalMs = 40;  // ~25Hz

// Rows buffered in RAM before one fwrite+fflush -- decouples sample rate
// from card-write rate, which is what bounds SD wear.
inline constexpr std::uint16_t kSdLoggerRowsPerFlush = 10;  // flush roughly every 400ms

// Reserved once at construction so appends between flushes never reallocate.
inline constexpr std::size_t kSdLoggerBufferReserveBytes = 8192;

inline constexpr const char* kSdLoggerDirectory = "/usd";
// 8.3-safe basename -- FatFS on the V5 may not support long file names.
inline constexpr const char* kSdLoggerFilenameFormat = "%s/lslog%03u.csv";
inline constexpr unsigned int kSdLoggerMaxFileIndex = 1000;

// -- Dashboard --
inline constexpr std::uint32_t kDashboardLoopPeriodMs = 125;  // ~8Hz

// -- Serial link (bench-tuning only, see serial_link.hpp) --
inline constexpr std::uint32_t kSerialLinkLoopPeriodMs = 100;  // ~10Hz

}  // namespace lightspeed::telemetry
