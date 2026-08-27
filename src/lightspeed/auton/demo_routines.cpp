#include "lightspeed/auton/demo_routines.hpp"

#include <vector>

#include "lightspeed/auton/sequencer.hpp"
#include "lightspeed/motion/path.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::auton::demo {

namespace {

// PLACEHOLDER -- see this file's header. Fires the demo arm toward its
// "HIGH" preset without blocking (Step 4 subsystem commands already run
// asynchronously via the scheduler), drives 24in forward concurrently,
// waits for the arm to finish before turning, then turns to face 90deg.
void runDemoStraightAndTurn(AutonomousContext& ctx) {
    fireAndForget([&] { ctx.exampleArm.moveToPreset("HIGH"); });

    ctx.driveStraightDistance.run(24.0);

    waitUntilSettled([&] { return ctx.exampleArm.getState() == subsystem::demo::ExampleArmState::holding; }, 2.0);

    ctx.turnToHeading.run(90.0);
}

// PLACEHOLDER -- see this file's header. A short pure-pursuit path,
// relative to wherever the robot currently is (i.e. wherever the selected
// start location put it).
void runDemoPursuitPath(AutonomousContext& ctx) {
    const odom::Pose start = ctx.odometry.getPose();
    const std::vector<motion::Waypoint> path{
        {start.xInches, start.yInches},
        {start.xInches + 12.0, start.yInches + 24.0},
        {start.xInches + 24.0, start.yInches + 24.0},
    };
    ctx.purePursuit.follow(path);
}

}  // namespace

Routine makeDemoStraightAndTurnRoutine() {
    return Routine{
        .name = "Demo: Straight + Turn",
        .validStartLocationIndices = {},  // valid from any start location
        // Approximate preview relative to Placeholder Start A (12,12,0deg):
        // 24in straight ahead. Purely illustrative -- the routine itself
        // works relative to wherever it's actually started.
        .previewPoints = {{12.0, 12.0}, {12.0, 36.0}},
        .run = runDemoStraightAndTurn,
    };
}

Routine makeDemoPursuitPathRoutine() {
    return Routine{
        .name = "Demo: Pursuit Path",
        .validStartLocationIndices = {},
        .previewPoints = {{12.0, 12.0}, {24.0, 36.0}, {36.0, 36.0}},
        .run = runDemoPursuitPath,
    };
}

}  // namespace lightspeed::auton::demo
