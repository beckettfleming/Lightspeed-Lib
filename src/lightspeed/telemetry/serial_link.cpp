#include "lightspeed/telemetry/serial_link.hpp"

#include <cstdio>

#include "lightspeed/telemetry/telemetry_constants.hpp"

namespace lightspeed::telemetry {

namespace {

void printCell(const ChannelSnapshot& channel) {
    switch (channel.type) {
        case ChannelType::number:
            std::printf(",%.4f", channel.number);
            break;
        case ChannelType::integer:
            std::printf(",%ld", static_cast<long>(channel.integer));
            break;
        case ChannelType::boolean:
            std::printf(",%d", channel.boolean ? 1 : 0);
            break;
        case ChannelType::text:
            std::printf(",%s", channel.text);
            break;
    }
}

}  // namespace

SerialLink::SerialLink(const TelemetryBus& bus) : bus_(bus) {}

SerialLink::~SerialLink() {
    if (task_.has_value()) {
        task_->remove();
    }
}

void SerialLink::start() {
    if (task_.has_value()) {
        return;
    }
    task_.emplace([this] { linkLoop(); }, "lightspeed_serial_link");
}

void SerialLink::linkLoop() {
    std::uint32_t previousTime = pros::millis();

    while (true) {
        pros::Task::delay_until(&previousTime, kSerialLinkLoopPeriodMs);

        const TelemetryBus::Snapshot snapshot = bus_.getSnapshot();

        if (!headerPrinted_) {
            std::printf("timestampMs");
            for (std::uint8_t i = 0; i < snapshot.count; ++i) {
                std::printf(",%s", snapshot.channels[i].name);
            }
            std::printf("\n");
            headerColumnCount_ = snapshot.count;
            headerPrinted_ = true;
        }

        const std::uint8_t columnCount = snapshot.count < headerColumnCount_ ? snapshot.count : headerColumnCount_;

        std::printf("%lu", static_cast<unsigned long>(pros::millis()));
        for (std::uint8_t i = 0; i < columnCount; ++i) {
            printCell(snapshot.channels[i]);
        }
        std::printf("\n");
    }
}

}  // namespace lightspeed::telemetry
