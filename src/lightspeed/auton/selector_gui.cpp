#include "lightspeed/auton/selector_gui.hpp"

#include <cstdint>
#include <vector>

#include "lightspeed/auton/auton_constants.hpp"
#include "lightspeed/auton/field_renderer.hpp"
#include "lightspeed/auton/screen_layout.hpp"
#include "lightspeed/auton/start_location.hpp"
#include "pros/screen.hpp"

namespace lightspeed::auton {

namespace {
constexpr std::uint32_t kPollIntervalMs = 50;
}  // namespace

SelectorGui::SelectorGui(odom::OdometryFusion& odometry, const RoutineRegistry& registry)
    : odometry_(odometry), registry_(registry) {}

SelectorGui::~SelectorGui() {
    stop();
}

void SelectorGui::start() {
    if (task_.has_value()) {
        return;
    }
    task_.emplace([this] { guiLoop(); }, "lightspeed_selector_gui");
}

void SelectorGui::stop() {
    if (!task_.has_value()) {
        return;
    }
    task_->remove();
    task_.reset();
}

const Routine* SelectorGui::getConfirmedRoutine() const {
    if (!confirmedRoutineIndex_.has_value()) {
        return nullptr;
    }
    return &registry_.getAll()[*confirmedRoutineIndex_];
}

const Routine* SelectorGui::getRoutineForAutonomous(bool* outUsedFallback) const {
    if (outUsedFallback != nullptr) {
        *outUsedFallback = false;
    }
    if (confirmedRoutineIndex_.has_value()) {
        return &registry_.getAll()[*confirmedRoutineIndex_];
    }
    if (tentativeRoutineIndex_.has_value()) {
        if (outUsedFallback != nullptr) {
            *outUsedFallback = true;
        }
        return &registry_.getAll()[*tentativeRoutineIndex_];
    }
    return nullptr;
}

void SelectorGui::guiLoop() {
    bool needsRedraw = true;
    std::int32_t lastHandledPressCount = -1;

    while (true) {
        if (needsRedraw) {
            if (currentScreen_ == Screen::startLocationPicker) {
                drawScreen1();
            } else {
                drawScreen2();
            }
            needsRedraw = false;
        }

        const pros::screen_touch_status_s_t touch = pros::screen::touch_status();
        if (touch.touch_status == pros::E_TOUCH_PRESSED && touch.press_count != lastHandledPressCount) {
            lastHandledPressCount = touch.press_count;
            needsRedraw = currentScreen_ == Screen::startLocationPicker ? handleTouchScreen1(touch.x, touch.y)
                                                                         : handleTouchScreen2(touch.x, touch.y);
        }

        pros::delay(kPollIntervalMs);
    }
}

void SelectorGui::drawScreen1() const {
    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(0, 0, kScreenWidth, kScreenHeight);

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, kTitleLine, "Select start location");

    renderField(kFieldToScreenConfig, kFieldDimensions, kScreen1FieldRegion);

    for (std::size_t i = 0; i < kStartLocations.size(); ++i) {
        const StartLocation& location = kStartLocations[i];
        const ScreenPoint p = fieldToScreen(location.pose.xInches, location.pose.yInches, kFieldToScreenConfig, kScreen1FieldRegion);
        const bool selected = startLocationIndex_.has_value() && *startLocationIndex_ == i;

        pros::screen::set_pen(selected ? pros::Color::lime : pros::Color::yellow);
        pros::screen::fill_circle(p.x, p.y, kStartZoneRadiusPixels);

        pros::screen::set_pen(pros::Color::white);
        pros::screen::print(pros::E_TEXT_MEDIUM, p.x + kStartZoneRadiusPixels + 4, p.y - 8, "%s", location.name);
    }
}

bool SelectorGui::handleTouchScreen1(std::int16_t x, std::int16_t y) {
    for (std::size_t i = 0; i < kStartLocations.size(); ++i) {
        const StartLocation& location = kStartLocations[i];
        const ScreenPoint p = fieldToScreen(location.pose.xInches, location.pose.yInches, kFieldToScreenConfig, kScreen1FieldRegion);
        const std::int32_t dx = x - p.x;
        const std::int32_t dy = y - p.y;
        if (dx * dx + dy * dy <= kStartZoneRadiusPixels * kStartZoneRadiusPixels) {
            startLocationIndex_ = i;
            odometry_.setPose(location.pose);

            tentativeRoutineIndex_.reset();
            confirmedRoutineIndex_.reset();
            scrollOffset_ = 0;
            currentScreen_ = Screen::routinePicker;
            return true;
        }
    }
    return false;
}

void SelectorGui::drawScreen2() const {
    pros::screen::set_pen(pros::Color::black);
    pros::screen::fill_rect(0, 0, kScreenWidth, kScreenHeight);

    pros::screen::set_pen(pros::Color::white);
    pros::screen::print(pros::E_TEXT_MEDIUM, kTitleLine, "Select routine: %s", kStartLocations[*startLocationIndex_].name);

    FieldToScreenConfig previewConfig = kFieldToScreenConfig;
    previewConfig.pixelsPerInch *= kPreviewScaleFactor;
    renderField(previewConfig, kFieldDimensions, kPreviewRegion);
    if (tentativeRoutineIndex_.has_value()) {
        renderRoute(previewConfig, kPreviewRegion, registry_.getAll()[*tentativeRoutineIndex_].previewPoints);
    }

    const std::vector<std::size_t> indices = registry_.getIndicesForStartLocation(*startLocationIndex_);

    for (std::uint8_t row = 0; row < kMaxVisibleRoutineRows; ++row) {
        const std::size_t listPos = scrollOffset_ + row;
        if (listPos >= indices.size()) {
            break;
        }
        const std::size_t routineIndex = indices[listPos];
        const std::int16_t rowY0 = kRoutineListY0 + row * kRoutineRowHeight;
        const std::int16_t rowY1 = rowY0 + kRoutineRowHeight - 2;

        const bool isTentative = tentativeRoutineIndex_.has_value() && *tentativeRoutineIndex_ == routineIndex;
        const bool isConfirmed = confirmedRoutineIndex_.has_value() && *confirmedRoutineIndex_ == routineIndex;

        pros::screen::set_pen(isConfirmed ? pros::Color::lime : (isTentative ? pros::Color::yellow : pros::Color::gray));
        pros::screen::draw_rect(kRoutineListX0, rowY0, kRoutineListX1, rowY1);
        if (isTentative || isConfirmed) {
            pros::screen::fill_rect(kRoutineListX0, rowY0, kRoutineListX1, rowY1);
        }

        pros::screen::set_pen((isTentative || isConfirmed) ? pros::Color::black : pros::Color::white);
        pros::screen::print(pros::E_TEXT_MEDIUM, kRoutineListX0 + 6, rowY0 + 6, "%s", registry_.getAll()[routineIndex].name);
    }

    if (indices.size() > kMaxVisibleRoutineRows) {
        pros::screen::set_pen(scrollOffset_ > 0 ? pros::Color::white : pros::Color::gray);
        pros::screen::draw_rect(kScrollUpZone.x0, kScrollUpZone.y0, kScrollUpZone.x1, kScrollUpZone.y1);
        pros::screen::print(pros::E_TEXT_MEDIUM, kScrollUpZone.x0 + 10, kScrollUpZone.y0 + 6, "^");

        const bool canScrollDown = static_cast<std::size_t>(scrollOffset_ + kMaxVisibleRoutineRows) < indices.size();
        pros::screen::set_pen(canScrollDown ? pros::Color::white : pros::Color::gray);
        pros::screen::draw_rect(kScrollDownZone.x0, kScrollDownZone.y0, kScrollDownZone.x1, kScrollDownZone.y1);
        pros::screen::print(pros::E_TEXT_MEDIUM, kScrollDownZone.x0 + 10, kScrollDownZone.y0 + 6, "v");
    }

    pros::screen::set_pen(pros::Color::white);
    pros::screen::draw_rect(kBackZone.x0, kBackZone.y0, kBackZone.x1, kBackZone.y1);
    pros::screen::print(pros::E_TEXT_MEDIUM, kBackZone.x0 + 10, kBackZone.y0 + 8, "Back");

    if (tentativeRoutineIndex_.has_value()) {
        const bool confirmed = confirmedRoutineIndex_.has_value() && *confirmedRoutineIndex_ == *tentativeRoutineIndex_;
        pros::screen::set_pen(confirmed ? pros::Color::lime : pros::Color::orange);
        pros::screen::draw_rect(kConfirmZone.x0, kConfirmZone.y0, kConfirmZone.x1, kConfirmZone.y1);
        if (confirmed) {
            pros::screen::fill_rect(kConfirmZone.x0, kConfirmZone.y0, kConfirmZone.x1, kConfirmZone.y1);
            pros::screen::set_pen(pros::Color::black);
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, kConfirmZone.x0 + 6, kConfirmZone.y0 + 8, confirmed ? "Confirmed" : "Confirm");
    }
}

bool SelectorGui::handleTouchScreen2(std::int16_t x, std::int16_t y) {
    if (insideRegion(x, y, kBackZone)) {
        currentScreen_ = Screen::startLocationPicker;
        tentativeRoutineIndex_.reset();
        confirmedRoutineIndex_.reset();
        return true;
    }

    const std::vector<std::size_t> indices = registry_.getIndicesForStartLocation(*startLocationIndex_);

    if (indices.size() > kMaxVisibleRoutineRows) {
        if (insideRegion(x, y, kScrollUpZone) && scrollOffset_ > 0) {
            --scrollOffset_;
            return true;
        }
        if (insideRegion(x, y, kScrollDownZone) && static_cast<std::size_t>(scrollOffset_ + kMaxVisibleRoutineRows) < indices.size()) {
            ++scrollOffset_;
            return true;
        }
    }

    if (tentativeRoutineIndex_.has_value() && insideRegion(x, y, kConfirmZone)) {
        confirmedRoutineIndex_ = tentativeRoutineIndex_;
        return true;
    }

    for (std::uint8_t row = 0; row < kMaxVisibleRoutineRows; ++row) {
        const std::size_t listPos = scrollOffset_ + row;
        if (listPos >= indices.size()) {
            break;
        }
        const std::int16_t rowY0 = kRoutineListY0 + row * kRoutineRowHeight;
        const std::int16_t rowY1 = rowY0 + kRoutineRowHeight - 2;
        const ScreenRegion rowZone{.x0 = kRoutineListX0, .y0 = rowY0, .x1 = kRoutineListX1, .y1 = rowY1};
        if (insideRegion(x, y, rowZone)) {
            // Tapping a different row swaps the preview non-destructively;
            // it does not touch confirmedRoutineIndex_.
            tentativeRoutineIndex_ = indices[listPos];
            return true;
        }
    }

    return false;
}

}  // namespace lightspeed::auton
