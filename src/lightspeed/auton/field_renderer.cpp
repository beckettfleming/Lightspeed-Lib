#include "lightspeed/auton/field_renderer.hpp"

#include "pros/screen.hpp"

namespace lightspeed::auton {

void renderField(const FieldToScreenConfig& config, const FieldDimensions& field, const ScreenRegion& region) {
    pros::screen::set_pen(pros::Color::silver);
    pros::screen::draw_rect(region.x0, region.y0, region.x1, region.y1);

    // Center divider, splitting the field into two alliance halves.
    const ScreenPoint top = fieldToScreen(field.widthInches / 2.0, field.lengthInches, config, region);
    const ScreenPoint bottom = fieldToScreen(field.widthInches / 2.0, 0.0, config, region);
    pros::screen::draw_line(top.x, top.y, bottom.x, bottom.y);
}

void renderRoute(const FieldToScreenConfig& config, const ScreenRegion& region, const std::vector<motion::Waypoint>& points) {
    if (points.empty()) {
        return;
    }

    pros::screen::set_pen(pros::Color::cyan);

    ScreenPoint previous = fieldToScreen(points.front().x, points.front().y, config, region);
    pros::screen::fill_circle(previous.x, previous.y, 2);

    for (std::size_t i = 1; i < points.size(); ++i) {
        const ScreenPoint current = fieldToScreen(points[i].x, points[i].y, config, region);
        pros::screen::draw_line(previous.x, previous.y, current.x, current.y);
        pros::screen::fill_circle(current.x, current.y, 2);
        previous = current;
    }
}

}  // namespace lightspeed::auton
