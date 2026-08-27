#include "lightspeed/telemetry/sd_logger.hpp"

#include <cstdio>

#include "lightspeed/telemetry/telemetry_constants.hpp"
#include "pros/misc.hpp"

namespace lightspeed::telemetry {

namespace {

// One below TASK_PRIORITY_DEFAULT: SD writes are blocking I/O and must
// never contend for CPU time against the drivetrain/odometry control tasks
// running at the default priority.
constexpr std::uint32_t kSdLoggerTaskPriority = TASK_PRIORITY_DEFAULT - 1;

void appendCell(std::string& row, const ChannelSnapshot& channel) {
    char field[32];
    switch (channel.type) {
        case ChannelType::number:
            std::snprintf(field, sizeof(field), "%.4f", channel.number);
            row += field;
            break;
        case ChannelType::integer:
            std::snprintf(field, sizeof(field), "%ld", static_cast<long>(channel.integer));
            row += field;
            break;
        case ChannelType::boolean:
            row += channel.boolean ? '1' : '0';
            break;
        case ChannelType::text:
            row += channel.text;
            break;
    }
}

}  // namespace

SdLogger::SdLogger(const TelemetryBus& bus) : bus_(bus) {
    rowBuffer_.reserve(kSdLoggerBufferReserveBytes);
}

SdLogger::~SdLogger() {
    if (task_.has_value()) {
        task_->remove();
    }
    flushIfDue(true);
    if (file_ != nullptr) {
        std::fclose(file_);
    }
}

void SdLogger::start() {
    if (task_.has_value()) {
        return;
    }
    task_.emplace([this] { loggerLoop(); }, kSdLoggerTaskPriority, TASK_STACK_DEPTH_DEFAULT, "lightspeed_sd_logger");
}

void SdLogger::startNewLogFile() {
    if (file_ != nullptr) {
        flushIfDue(true);
        std::fclose(file_);
        file_ = nullptr;
    }
    headerWritten_ = false;
    headerColumnCount_ = 0;
    rowBuffer_.clear();
    bufferedRowCount_ = 0;

    char path[64];
    for (unsigned int index = 0; index < kSdLoggerMaxFileIndex; ++index) {
        std::snprintf(path, sizeof(path), kSdLoggerFilenameFormat, kSdLoggerDirectory, index);

        std::FILE* existing = std::fopen(path, "r");
        if (existing != nullptr) {
            std::fclose(existing);
            continue;  // already used by a previous run -- never overwrite it
        }

        file_ = std::fopen(path, "w");
        if (file_ == nullptr) {
            // No SD card present (or not writable) -- give up rather than
            // spin through the remaining indices, they'd all fail the same
            // way. Logging is simply unavailable for this run.
            std::printf("[SdLogger] WARNING: could not open '%s' -- SD logging disabled this run\n", path);
        } else {
            std::printf("[SdLogger] logging to '%s'\n", path);
        }
        return;
    }
    std::printf("[SdLogger] WARNING: no unused log filename found (checked %u) -- SD logging disabled this run\n",
                kSdLoggerMaxFileIndex);
}

void SdLogger::appendRow(const TelemetryBus::Snapshot& snapshot) {
    if (!headerWritten_) {
        rowBuffer_ += "timestampMs";
        for (std::uint8_t i = 0; i < snapshot.count; ++i) {
            rowBuffer_ += ',';
            rowBuffer_ += snapshot.channels[i].name;
        }
        rowBuffer_ += '\n';
        headerColumnCount_ = snapshot.count;
        headerWritten_ = true;
    }

    // Channel registration is append-only and stabilizes within the first
    // few cycles of a run (see telemetry_bus.hpp); clamping to the header's
    // column count keeps every row the same width even if a channel were to
    // register slightly after the header snapshot was taken.
    const std::uint8_t columnCount = snapshot.count < headerColumnCount_ ? snapshot.count : headerColumnCount_;

    char timestampField[16];
    std::snprintf(timestampField, sizeof(timestampField), "%lu", static_cast<unsigned long>(pros::millis()));
    rowBuffer_ += timestampField;
    for (std::uint8_t i = 0; i < columnCount; ++i) {
        rowBuffer_ += ',';
        appendCell(rowBuffer_, snapshot.channels[i]);
    }
    rowBuffer_ += '\n';
    ++bufferedRowCount_;
}

void SdLogger::flushIfDue(bool force) {
    if (file_ == nullptr || rowBuffer_.empty()) {
        return;
    }
    if (!force && bufferedRowCount_ < kSdLoggerRowsPerFlush) {
        return;
    }

    const std::size_t written = std::fwrite(rowBuffer_.data(), 1, rowBuffer_.size(), file_);
    const bool flushOk = std::fflush(file_) == 0;
    if (written != rowBuffer_.size() || !flushOk) {
        // Card pulled (or failed) mid-run: this batch of rows is lost, and
        // retrying every cycle against a now-dead file just wastes cycles
        // for the rest of the run -- close it and stop logging rather than
        // silently failing forever.
        std::printf("[SdLogger] WARNING: write failed -- SD logging disabled for the rest of this run\n");
        std::fclose(file_);
        file_ = nullptr;
    }
    rowBuffer_.clear();  // keeps reserved capacity -- no reallocation next cycle
    bufferedRowCount_ = 0;
}

void SdLogger::loggerLoop() {
    std::uint32_t previousTime = pros::millis();

    while (true) {
        pros::Task::delay_until(&previousTime, kSdLoggerSampleIntervalMs);

        const bool disabledNow = pros::competition::is_disabled();
        if (previouslyDisabled_ && !disabledNow) {
            startNewLogFile();
        }
        previouslyDisabled_ = disabledNow;

        if (file_ == nullptr) {
            continue;
        }

        appendRow(bus_.getSnapshot());
        flushIfDue(false);
    }
}

}  // namespace lightspeed::telemetry
