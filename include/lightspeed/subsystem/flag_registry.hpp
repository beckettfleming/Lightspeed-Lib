/**
 * \file lightspeed/subsystem/flag_registry.hpp
 *
 * Lightweight named boolean store. Subsystems register flags (e.g.
 * "exampleArm.isExtended") at init and update them from their own update();
 * driver control's accel-limit table reads them by name. Registration makes
 * a name mismatch detectable via a false return instead of silently doing
 * nothing.
 *
 * Thread-safe: written from the scheduler task, read from any other.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/subsystem/
 */

#pragma once

#include <array>
#include <cstdint>

#include "pros/rtos.hpp"

namespace lightspeed::subsystem {

class FlagRegistry {
public:
    static FlagRegistry& instance();

    FlagRegistry(const FlagRegistry&) = delete;
    FlagRegistry& operator=(const FlagRegistry&) = delete;

    // False (and logs) if the name is already registered or the registry is
    // full. A caller that ignores this return value is NOT protected from
    // cross-subsystem name collisions -- see setFlag().
    bool registerFlag(const char* name, bool initialValue = false);

    // Swap-remove, so getNameAt() ordering is stable except across this
    // call. Intended for teardown/test cleanup.
    bool unregisterFlag(const char* name);

    // False if name was never registered, which catches a typo'd name
    // instead of it silently doing nothing.
    //
    // NOTE -- no ownership enforcement: any caller knowing a flag's name can
    // set it. registerFlag() rejects a SECOND registration of the same name,
    // but if that failure is ignored the second subsystem will still
    // setFlag() every cycle, stomping the first's value. Give every
    // subsystem's flags a unique subsystem-prefixed name to avoid this.
    bool setFlag(const char* name, bool value);

    // defaultValue if name was never registered.
    [[nodiscard]] bool getFlag(const char* name, bool defaultValue = false) const;

    [[nodiscard]] bool isRegistered(const char* name) const;

    [[nodiscard]] std::uint8_t getRegisteredCount() const;

    // nullptr if index is out of range.
    [[nodiscard]] const char* getNameAt(std::uint8_t index) const;

private:
    FlagRegistry() = default;

    struct Entry {
        const char* name = nullptr;
        bool value = false;
    };

    // Caller must hold mutex_.
    [[nodiscard]] int findIndexLocked(const char* name) const;

    static constexpr std::uint8_t kMaxFlags = 16;
    std::array<Entry, kMaxFlags> entries_{};
    std::uint8_t count_ = 0;
    mutable pros::Mutex mutex_;
};

}  // namespace lightspeed::subsystem
