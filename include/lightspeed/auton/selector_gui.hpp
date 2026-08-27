/**
 * \file lightspeed/auton/selector_gui.hpp
 *
 * Two-screen touch GUI selector on the V5 brain screen: screen 1 picks a
 * start location (and sets odometry's pose), screen 2 picks a routine
 * (filtered to that start, with a live route preview) and locks it in.
 * Owns a single background task that polls touches and redraws -- the
 * only task that should touch pros::screen while this is running (call
 * stop() before anything else needs the screen, e.g. a future telemetry
 * dashboard, or once autonomous() is about to run the confirmed routine).
 *
 * Stays live/re-editable the whole time it's running: nothing is locked in
 * until the confirm zone is tapped, and the back zone can always return to
 * screen 1 and reset screen 2's state.
 */

#pragma once

#include <cstdint>
#include <optional>

#include "lightspeed/auton/routine.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "pros/rtos.hpp"

namespace lightspeed::auton {

class SelectorGui {
public:
    SelectorGui(odom::OdometryFusion& odometry, const RoutineRegistry& registry);
    ~SelectorGui();

    SelectorGui(const SelectorGui&) = delete;
    SelectorGui& operator=(const SelectorGui&) = delete;

    // Starts the GUI's background task (drawing + touch polling). No-op if
    // already started.
    void start();

    // Stops the GUI's background task. No-op if not running. Call this
    // before anything else draws to the screen.
    void stop();

    // The routine confirmed via screen 2's confirm zone, or nullptr if
    // none has been confirmed yet.
    [[nodiscard]] const Routine* getConfirmedRoutine() const;

    // Fallback-aware variant for the competition entry point: returns the
    // confirmed routine if one was tapped; otherwise, if a routine was at
    // least tentatively previewed on screen 2 (tapped once but Confirm was
    // never tapped), returns that instead -- on the assumption the driver
    // picked it deliberately and simply forgot to lock it in before the
    // match timer started, rather than the robot doing nothing for the
    // whole autonomous period. Returns nullptr only if nothing was even
    // tentatively selected. outUsedFallback, if non-null, is set to true
    // when the tentative-not-confirmed path was taken, so a caller can
    // log/telemetry the distinction rather than it being silent.
    [[nodiscard]] const Routine* getRoutineForAutonomous(bool* outUsedFallback = nullptr) const;

private:
    enum class Screen { startLocationPicker, routinePicker };

    void guiLoop();

    void drawScreen1() const;
    void drawScreen2() const;

    // Return true if handling the touch changed GUI state (redraw needed).
    bool handleTouchScreen1(std::int16_t x, std::int16_t y);
    bool handleTouchScreen2(std::int16_t x, std::int16_t y);

    odom::OdometryFusion& odometry_;
    const RoutineRegistry& registry_;

    Screen currentScreen_ = Screen::startLocationPicker;
    std::optional<std::size_t> startLocationIndex_;
    std::optional<std::size_t> tentativeRoutineIndex_;   // index into registry_.getAll()
    std::optional<std::size_t> confirmedRoutineIndex_;   // index into registry_.getAll()
    std::uint8_t scrollOffset_ = 0;                      // first visible row, in the filtered list

    std::optional<pros::Task> task_;
};

}  // namespace lightspeed::auton
