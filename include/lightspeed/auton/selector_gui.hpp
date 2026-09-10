/**
 * \file lightspeed/auton/selector_gui.hpp
 *
 * Two-screen touch GUI selector: screen 1 picks a start location (and sets
 * odometry's pose), screen 2 picks a routine filtered to that start and
 * locks it in. Owns a background task and is the ONLY task that may touch
 * pros::screen while running -- call stop() before anything else draws.
 *
 * Nothing is locked in until the confirm zone is tapped.
 *
 * Docs: https://beckettfleming.github.io/Lightspeed-Lib/layers/autonomous/
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

    // No-op if already started.
    void start();

    // No-op if not running. Call before anything else draws to the screen.
    void stop();

    // nullptr if nothing has been confirmed.
    [[nodiscard]] const Routine* getConfirmedRoutine() const;

    // Fallback-aware variant for the competition entry point: the confirmed
    // routine, else the last tentatively-previewed one (on the assumption
    // the driver picked it deliberately and forgot to lock it in, rather
    // than the robot doing nothing all period), else nullptr.
    // outUsedFallback reports which path was taken so the caller can log it.
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
