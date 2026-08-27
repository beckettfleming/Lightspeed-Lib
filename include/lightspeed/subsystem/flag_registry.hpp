/**
 * \file lightspeed/subsystem/flag_registry.hpp
 *
 * Lightweight named boolean store. Subsystems register their own flags
 * (e.g. "exampleArm.isExtended") at init and update them from their own
 * update() -- nothing else consumes these yet, but a future driver-control
 * accel-limiting feature will read them by name. Registration makes a
 * mismatch between the name a consumer queries and what a subsystem
 * actually registered detectable (via a false return) instead of silently
 * doing nothing.
 *
 * Thread-safe: subsystems update flags from the scheduler task while other
 * tasks (bench harness now, driver control later) may read them at any
 * time.
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

    // Declares a flag by name with an initial value. Returns false (and
    // logs) if the name is already registered or the registry is full. A
    // caller that ignores this return value and proceeds to call setFlag()
    // anyway is NOT protected from cross-subsystem name collisions -- see
    // setFlag()'s doc comment.
    bool registerFlag(const char* name, bool initialValue = false);

    // Removes a previously registered flag by name (swap-remove, so
    // enumeration order via getNameAt() is stable except across this
    // call). Returns false if it wasn't registered. Intended for teardown/
    // test cleanup -- most subsystems live for the program's lifetime and
    // never need this.
    bool unregisterFlag(const char* name);

    // Sets a registered flag's value. Returns false if name was never
    // registered -- lets a caller detect a typo'd/mismatched flag name
    // instead of it silently doing nothing.
    //
    // NOTE -- no ownership enforcement: any caller that knows a flag's name
    // can set it, regardless of which subsystem originally registered it.
    // registerFlag() only rejects a SECOND registration of the same name
    // (returns false, doesn't overwrite the first registrant's slot) -- but
    // if that failure is ignored (as e.g. a constructor discarding the
    // return value would), the second subsystem still believes it owns the
    // name and will happily setFlag() it every cycle, silently stomping the
    // first subsystem's value. There is currently only one real flag/
    // subsystem in this codebase (exampleArm.isExtended), so this is a
    // documented risk for when a second subsystem is added, not a fix --
    // give every subsystem's flags a unique, subsystem-prefixed name (as
    // exampleArm's already does) to avoid it in practice.
    bool setFlag(const char* name, bool value);

    // Returns defaultValue if name was never registered.
    [[nodiscard]] bool getFlag(const char* name, bool defaultValue = false) const;

    [[nodiscard]] bool isRegistered(const char* name) const;

    // Number of currently registered flags, for enumeration via getNameAt()
    // -- e.g. a future debug dashboard listing every flag by name.
    [[nodiscard]] std::uint8_t getRegisteredCount() const;

    // The name registered at `index` (0..getRegisteredCount()), or nullptr
    // if index is out of range.
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
