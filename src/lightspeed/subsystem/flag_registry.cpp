#include "lightspeed/subsystem/flag_registry.hpp"

#include <cstdio>
#include <cstring>

#include "lightspeed/telemetry/telemetry_bus.hpp"

namespace lightspeed::subsystem {

FlagRegistry& FlagRegistry::instance() {
    static FlagRegistry registry;
    return registry;
}

int FlagRegistry::findIndexLocked(const char* name) const {
    for (std::uint8_t i = 0; i < count_; ++i) {
        if (std::strcmp(entries_[i].name, name) == 0) {
            return i;
        }
    }
    return -1;
}

bool FlagRegistry::registerFlag(const char* name, bool initialValue) {
    mutex_.take();

    if (findIndexLocked(name) >= 0) {
        mutex_.give();
        std::printf("[lightspeed::subsystem] WARNING: flag '%s' is already registered.\n", name);
        return false;
    }
    if (count_ >= kMaxFlags) {
        mutex_.give();
        std::printf("[lightspeed::subsystem] WARNING: FlagRegistry is full (%u max) -- '%s' was not registered.\n",
                     kMaxFlags, name);
        return false;
    }

    entries_[count_] = Entry{name, initialValue};
    ++count_;

    mutex_.give();
    return true;
}

bool FlagRegistry::unregisterFlag(const char* name) {
    mutex_.take();
    const int index = findIndexLocked(name);
    if (index >= 0) {
        // Swap-remove: move the last entry into the removed slot so the
        // array stays contiguous without shifting everything after it.
        entries_[static_cast<std::uint8_t>(index)] = entries_[count_ - 1];
        entries_[count_ - 1] = Entry{};
        --count_;
    }
    mutex_.give();
    return index >= 0;
}

std::uint8_t FlagRegistry::getRegisteredCount() const {
    mutex_.take();
    const std::uint8_t count = count_;
    mutex_.give();
    return count;
}

const char* FlagRegistry::getNameAt(std::uint8_t index) const {
    mutex_.take();
    const char* name = index < count_ ? entries_[index].name : nullptr;
    mutex_.give();
    return name;
}

bool FlagRegistry::setFlag(const char* name, bool value) {
    mutex_.take();
    const int index = findIndexLocked(name);
    if (index >= 0) {
        entries_[index].value = value;
    }
    mutex_.give();

    if (index >= 0) {
        // Forwarded here rather than at each subsystem's own call site --
        // every flag a subsystem registers becomes a telemetry channel for
        // free, present or future.
        telemetry::TelemetryBus::instance().record(name, value);
    }
    return index >= 0;
}

bool FlagRegistry::getFlag(const char* name, bool defaultValue) const {
    mutex_.take();
    const int index = findIndexLocked(name);
    const bool result = index >= 0 ? entries_[index].value : defaultValue;
    mutex_.give();
    return result;
}

bool FlagRegistry::isRegistered(const char* name) const {
    mutex_.take();
    const bool result = findIndexLocked(name) >= 0;
    mutex_.give();
    return result;
}

}  // namespace lightspeed::subsystem
