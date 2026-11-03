#include "lightspeed/auton/demo_routines.hpp"

#include <vector>

#include "lightspeed/auton/sequencer.hpp"
#include "lightspeed/motion/path.hpp"
#include "lightspeed/odom/types.hpp"

namespace lightspeed::auton::demo {

namespace {

// PLACEHOLDER -- see this file's header. Drives 24in forward, then turns to
// face 90deg.
void runDemoStraightAndTurn(AutonomousContext& ctx) {
    ctx.driveStraightDistance.run(24.0);
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

void runScoreOneClose(AutonomousContext& ctx) 
{
    ctx.driveStraightDistance.run(24);
    ctx.driveStraightDistance.run(-24);
    ctx.driveStraightDistance.run(23);
    ctx.turnToHeading.run(270.0);
    //Hinge out
    //lift up
    ctx.driveStraightDistance.run(-22);
    //lift down
    //clamp off
    //lift up
    ctx.driveStraightDistance.run(6);
    //clamp on
    //lift down
    //hinge in

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

Routine makeScoreOneClose()
{
    return Routine
    {
        .name = "Score One Close",
        .validStartLocationIndices = {},
        .previewPoints = {{}, {}},
        .run = runScoreOneClose,
    };
}
}  // namespace lightspeed::auton::demo
