#include "lightspeed/telemetry/telemetry_bus.hpp"

#include <cstring>

namespace lightspeed::telemetry {

TelemetryBus& TelemetryBus::instance() {
    static TelemetryBus bus;
    return bus;
}

int TelemetryBus::findOrCreateIndexLocked(const char* name) {
    for (std::uint8_t i = 0; i < count_; ++i) {
        if (std::strcmp(channels_[i].name, name) == 0) {
            return i;
        }
    }
    if (count_ >= kMaxChannels) {
        // No console print here (unlike FlagRegistry's registration
        // warnings) -- record() runs every cycle on hot-path tasks
        // (drivetrain, odometry), and a genuinely full registry is a
        // fixed, compile-time-discoverable condition, not something that
        // needs runtime spam to diagnose.
        return -1;
    }
    channels_[count_].name = name;
    return count_++;
}

void TelemetryBus::record(const char* name, double value) {
    mutex_.take();
    const int index = findOrCreateIndexLocked(name);
    if (index >= 0) {
        channels_[index].type = ChannelType::number;
        channels_[index].number = value;
    }
    mutex_.give();
}

void TelemetryBus::record(const char* name, bool value) {
    mutex_.take();
    const int index = findOrCreateIndexLocked(name);
    if (index >= 0) {
        channels_[index].type = ChannelType::boolean;
        channels_[index].boolean = value;
    }
    mutex_.give();
}

void TelemetryBus::record(const char* name, std::int32_t value) {
    mutex_.take();
    const int index = findOrCreateIndexLocked(name);
    if (index >= 0) {
        channels_[index].type = ChannelType::integer;
        channels_[index].integer = value;
    }
    mutex_.give();
}

void TelemetryBus::record(const char* name, const char* value) {
    mutex_.take();
    const int index = findOrCreateIndexLocked(name);
    if (index >= 0) {
        channels_[index].type = ChannelType::text;
        channels_[index].text = value;
    }
    mutex_.give();
}

TelemetryBus::Snapshot TelemetryBus::getSnapshot() const {
    Snapshot snapshot;
    mutex_.take();
    snapshot.channels = channels_;
    snapshot.count = count_;
    mutex_.give();
    return snapshot;
}

}  // namespace lightspeed::telemetry
