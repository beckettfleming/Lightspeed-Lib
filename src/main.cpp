#include "main.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

#include "lightspeed/auton/auton_constants.hpp"
#include "lightspeed/auton/autonomous_context.hpp"
#include "lightspeed/auton/demo_routines.hpp"
#include "lightspeed/auton/routine.hpp"
#include "lightspeed/auton/selector_gui.hpp"
#include "lightspeed/auton/start_location.hpp"
#include "lightspeed/control/drivetrain_velocity_constants.hpp"
#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/control/slew_rate_limiter.hpp"
#include "lightspeed/diagnostics/diagnostic_mode.hpp"
#include "lightspeed/driver/accel_limit_resolver.hpp"
#include "lightspeed/driver/drive_mode.hpp"
#include "lightspeed/driver/driver_control_constants.hpp"
#include "lightspeed/driver/input_profile.hpp"
#include "lightspeed/hal/config.hpp"
#include "lightspeed/hal/imu.hpp"
#include "lightspeed/hal/motor_group.hpp"
#include "lightspeed/hal/rotation_sensor.hpp"
#include "lightspeed/motion/drive_straight_distance.hpp"
#include "lightspeed/motion/drive_to_point.hpp"
#include "lightspeed/motion/motion_constants.hpp"
#include "lightspeed/motion/move_to_pose.hpp"
#include "lightspeed/motion/pose_math.hpp"
#include "lightspeed/motion/pure_pursuit_controller.hpp"
#include "lightspeed/motion/turn_to_heading.hpp"
#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/imu_source.hpp"
#include "lightspeed/odom/odometry_constants.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/odom/tracking_wheel_source.hpp"
#include "lightspeed/odom/types.hpp"
#include "lightspeed/telemetry/dashboard.hpp"
#include "lightspeed/telemetry/sd_logger.hpp"
#include "lightspeed/telemetry/serial_link.hpp"
#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace {

// -- Shared, program-lifetime hardware/control objects --
//
// initialize() constructs these once; opcontrol() and autonomous() both
// reference the SAME instances. If opcontrol() built its own second
// DrivetrainVelocityController on the same ports, two independent PIDF loops
// would fight over the same physical motors.
std::optional<lightspeed::hal::MotorGroup> gLeftDrive;
std::optional<lightspeed::hal::MotorGroup> gRightDrive;
std::optional<lightspeed::hal::MotorGroup> gWinchLeft;
std::optional<lightspeed::hal::MotorGroup> gWinchRight;
std::optional<lightspeed::hal::MotorGroup> gIntake;
std::optional<pros::adi::DigitalOut> gClawPneumatic;
std::optional<pros::adi::DigitalOut> gHingePneumatic;
std::optional<pros::adi::DigitalIn> gLiftMinimumLimitSwitch;
std::optional<pros::adi::DigitalOut> gCalibratedIndicator;
std::optional<pros::adi::DigitalOut> gCalibrationIndicator;
bool gClawPneumaticExtended = false;
bool gHingePneumaticExtended = false;
double gWinchLeftStartingPosition = 0.0;
double gWinchRightStartingPosition = 0.0;
bool gWinchStoppedForNoLoad = false;
std::optional<std::uint32_t> gWinchLowCurrentSince;

std::optional<lightspeed::control::DrivetrainVelocityController> gDrivetrain;

std::optional<lightspeed::hal::Imu> gPrimaryImu;
std::optional<lightspeed::hal::Imu> gSecondaryImu;
std::optional<lightspeed::hal::RotationSensor> gOdometryPodRotation;
std::optional<lightspeed::odom::IMESource> gLeftIme;
std::optional<lightspeed::odom::IMESource> gRightIme;
std::optional<lightspeed::odom::IMUSource> gImuSource;
std::optional<lightspeed::odom::TrackingWheelSource> gOdometryPod;
std::optional<lightspeed::odom::OdometryFusion> gOdometry;

std::optional<lightspeed::motion::TurnToHeading> gTurnToHeading;
std::optional<lightspeed::motion::DriveStraightDistance> gDriveStraightDistance;
std::optional<lightspeed::motion::DriveToPoint> gDriveToPoint;
std::optional<lightspeed::motion::MoveToPose> gMoveToPose;
std::optional<lightspeed::motion::PurePursuitController> gPurePursuit;

std::optional<lightspeed::driver::InputProfile> gInputProfile;
std::optional<lightspeed::driver::AccelLimitResolver> gAccelLimitResolver;
std::optional<lightspeed::control::SlewRateLimiter> gLeftAccelSlew;
std::optional<lightspeed::control::SlewRateLimiter> gRightAccelSlew;
std::atomic<bool> gLimpModeEnabled{false};

lightspeed::auton::RoutineRegistry gRoutineRegistry;
std::optional<lightspeed::auton::SelectorGui> gSelectorGui;

// TelemetryBus is a singleton, so no shared bus object is constructed here.
std::optional<lightspeed::telemetry::SdLogger> gSdLogger;
std::optional<lightspeed::telemetry::Dashboard> gDashboard;
// Started only inside diagnostic mode, not during competition operation.
std::optional<lightspeed::telemetry::SerialLink> gSerialLink;

constexpr std::uint32_t kLoopPeriodMs = 10;  // ~100Hz driver-control loop
constexpr double kDtSeconds = kLoopPeriodMs / 1000.0;
constexpr std::uint32_t kStatusIntervalMs = 1000;  // periodic heartbeat, ~1Hz
constexpr std::uint32_t kWinchNoLoadConfirmMs = 200;
constexpr double kWinchNoLoadCurrentMilliAmps = 500.0;
constexpr double kWinchPositionToleranceDegrees = 1.0;
constexpr double kHingeAutoExtendLiftDegrees = 250.0;
constexpr std::int32_t kWinchVoltage = 12000;
constexpr double kWinchLoweringVoltageScaleBelowHingeThreshold = 0.7;
constexpr std::int32_t kIntakeVoltage = 12000;
constexpr double kMotionTestDistanceInches = 12.0;
constexpr std::int32_t kCloseAllianceDriveVoltageLimitMillivolts = 8000;
// Toggle bumps: 2in moves under the normal 60in/s^2 profile (and overshoot
// guard) are too gentle to flip the toggle after the 2026-11-19 gearing fix.
// Raise maxAcceleration for a harder hit; the voltage limit above still caps
// how hard it can push.
const lightspeed::motion::MotionProfileConfig kCloseAllianceToggleProfile{
    .maxVelocity = 40.0,
    .maxAcceleration = 100.0,  // 120 flipped it well but rocked the robot on each reversal
    .maxJerk = 0.0,  // plain trapezoid -- jerk-limiting just softens a 2in move
};
// Each toggle bump may end pressed against the toggle short of its distance
// (2026-11-19: third bump pinned at 1.49/2.5in, pushing for 4.5s until the
// default 5s timeout). A 2in bump takes ~0.3s, so this is plenty to finish.
constexpr double kCloseAllianceToggleTimeoutSeconds = 0.75;
// Toggle wiggle: push, back up, push again. The back-up was 1.75in -- not
// enough run-up to catch the second toggle (2026-11-19). Backing up 0.5in more
// gives the second push more speed when it arrives; the second push grows by
// the same 0.5in so it still reaches the same spot.
constexpr double kCloseAllianceToggleFirstPushInches = 2.0;
constexpr double kCloseAllianceToggleBackUpInches = 2.25;
constexpr double kCloseAllianceToggleSecondPushInches = 3.0;
// Pause after backing up, before the second push, so the toggle has time to
// reseat (2026-11-19: second push arrived ~immediately and missed the flip).
// The drivetrain holds 0 speed during it.
constexpr std::uint32_t kCloseAllianceToggleReseatMs = 300;
constexpr double kAutonomousLiftRaiseDegrees = 85.0;
constexpr std::int32_t kAutonomousLiftVelocityRpm = 100;
constexpr double kCloseAllianceGoalBackInches = 10;
// The goal physically stops the robot at left ~17.6 -- the right spot (confirmed
// 2026-11-19). 18.0 puts that within the 0.5in tolerance so the path settles
// there instead of pushing into the goal until its timeout (was 18.5: ~2.5s
// wasted pushing every run).
constexpr double kCloseAllianceGoalLeftInches = 18.0;
// How close (in) the goal path must get before it counts as arrived, and how
// long it may try. A stall can no longer eat the 8s default timeout.
constexpr double kCloseAllianceGoalToleranceInches = 0.5;
constexpr double kCloseAllianceGoalTimeoutSeconds = 3.0;
constexpr std::uint32_t kCloseAllianceLiftWaitTimeoutMs = 1000;
// Quarter-arc radius for the swing into the goal; the final straight reverse
// into the goal is (kCloseAllianceGoalLeftInches - this) long. Must be
// <= kCloseAllianceGoalLeftInches. Smaller = longer straight approach, tighter turn.
constexpr double kCloseAllianceTurnRadiusInches = 14.0;
// Shortest allowed straight reverse into the goal after the arc.
constexpr double kCloseAllianceMinFinalStraightInches = 4.0;
// Lookahead cap for the goal path. The default (up to 18in) is bigger than the
// arc itself, so the robot would cut the turn and arrive crooked.
constexpr double kCloseAllianceGoalLookaheadInches = 9.0;
// Before opening the claw at the goal: max wait for the lift, and how close
// counts as "up". Loose + short so scoring isn't held up by the lift's final
// settle.
constexpr std::uint32_t kCloseAllianceScoreLiftWaitTimeoutMs = 250;
constexpr double kCloseAllianceScoreLiftToleranceDegrees = 5.0;
// Pause after opening the claw before pulling away from the goal.
constexpr std::uint32_t kCloseAllianceClawReleaseMs = 500;
// Second leg: drive forward off the goal to this point (same start-relative
// frame as the goal: back of / left of the starting pose).
// Field ~(9, 31) from the Close Alliance start (0, 11.375, 180deg).
constexpr double kCloseAllianceSecondBackInches = 19.625;
constexpr double kCloseAllianceSecondLeftInches = 9.0;
// After the second leg: turn to start heading + this (clockwise-positive),
// and raise the lift this much further on top of kAutonomousLiftRaiseDegrees.
// 41 not 40: the turn reliably settles ~1deg short (219.0-219.3 vs 220 over
// several runs, inside its 1deg window), leaving the aim at pair 1 incomplete.
constexpr double kCloseAllianceTurnRelativeDegrees = 41.0;
constexpr double kCloseAllianceSecondLiftRaiseDegrees = 170.0;
// After the turn: drive straight backward this far.
constexpr double kCloseAllianceFinalReverseInches = 17.75;

// ===== Cup + pin pairs (after the final reverse) =====
// Lift heights, degrees above the power-on (bottom) position (the encoder
// reading the lift code uses). 8in / 0.625in spool radius = 733deg went far
// too high (2026-11-19), so the lift isn't 1:1 spool-on-motor (likely a
// multi-stage/cascade lift or gearing). MEASURE instead: in driver control,
// raise the lift to each height by hand; the terminal status line prints
// "lift=NNNdeg" once a second. TODO: replace these placeholder guesses.
constexpr double kPairPickupLiftDegrees = 0.0;  // lowered to grab a pair
constexpr double kPairScoreLiftDegrees = 350.0; // carried while backing in: clears the goal + scored pin
constexpr double kPairClearLiftDegrees = 400.0; // after releasing, so the claw lifts clear of the pair
constexpr std::uint32_t kPairLiftWaitTimeoutMs = 2500;  // 733deg at 100rpm is ~1.2s
constexpr double kPairLiftToleranceDegrees = 5.0;
constexpr std::uint32_t kPairGrabSettleMs = 300;  // after closing the claw, before lifting
// Goal approach for pairs: from the pickup point, turn so the back faces the
// goal's CENTER and reverse straight at it, stopping when the robot's center
// is kGoalContactDistanceInches from it -- the same spacing as when the pin
// was scored. (Aiming at the pin-scoring robot position instead, as before,
// only works approaching from the pin's direction; from anywhere else the
// goal ends up off to the side -- 2026-11-19, pair 1 landed well off.)
// Robot tracking center -> goal center at contact = how far behind the
// turning center the robot touches (2.375in, measured) + goal radius (6in).
constexpr double kRobotContactBehindCenterInches = 2.375;
constexpr double kGoalRadiusInches = 6.0;
constexpr double kGoalContactDistanceInches = kRobotContactBehindCenterInches + kGoalRadiusInches;
constexpr double kPairGoalTimeoutSeconds = 3.0;
// Field trim for each pair's goal approach (2026-11-19: pair 1 needed to back
// in further and sit a couple inches more to the right). "Right" = the
// robot's right while it backs in (facing away from the goal). Negative
// values go the other way. Pair 2 comes at the goal from a different side,
// so it gets its own trim. TODO: tune pair 2.
constexpr double kPairGoalExtraBackInches = 1.5;
constexpr double kPairGoalShiftRightInches = 2.0;
constexpr double kPair2GoalExtraBackInches = 3.0;  // 1.5 stopped short (2026-11-19)
constexpr double kPair2GoalShiftRightInches = 0.0;
// After scoring pair 1, drive forward this far off the goal before heading to
// pair 2, so the turn doesn't drag the claw/lift across the scored pair.
constexpr double kPairGoalPullOffInches = 6.0;
// Route to pair 2: reverse from the pull-off spot to this staging point first,
// then straight to the pickup. Reversing diagonally from the pull-off spot
// swung the robot's back straight into the goal (2026-11-19: hit it after
// ~4in). Staging = level with the pull-off spot (start-frame "back" ~23 ->
// field y ~34, well clear of the goal) and level with the pickup ("left"), so
// the last leg comes straight down onto the pair. TODO: confirm on the field
// with the driver-control pose readout.
constexpr double kPair2StagingBackInches = 23.0;
// Pickup trim. Was -6 for the old diagonal approach; the staging route comes
// in from a different direction, so "right" means something else now. Reset
// to 0 -- re-tune once the route is right.
constexpr double kPair2PickupShiftRightInches = 0.0;
// After the pin's curved path settles, push back this much more to seat onto
// the goal (the path stops at the edge of its tolerance, short of seated).
// Overshoot guard off: pressing into the goal is the point. Was 0.75 / 0.5s:
// still not seated far enough to score (2026-11-19).
constexpr double kGoalSeatPushInches = 1.5;
constexpr double kGoalSeatPushTimeoutSeconds = 0.75;
// Second pair pickup: tracking-center stop 24in left of the goal, measured in
// the same start-relative back/left frame as the goal coordinates.
constexpr bool kSecondPairPositionSet = true;
constexpr double kSecondPairBackInches = kCloseAllianceGoalBackInches;
constexpr double kSecondPairLeftInches = kCloseAllianceGoalLeftInches + 24.0;
const std::vector<std::size_t> kMotionTestStartLocationIndices{0, 1};

double normalizeStick(std::int32_t rawAnalog) {
	return static_cast<double>(rawAnalog) / 127.0;
}

// Non-blocking. raisedDegrees is measured from the lift's power-on (bottom)
// position, not from wherever it is now, so successive calls don't
// accumulate error and match waitForLiftRaisedDegrees().
bool commandLiftRaisedByEncoderDegrees(double raisedDegrees) {
	const std::int32_t leftResult =
	    gWinchLeft->moveAbsolute(gWinchLeftStartingPosition - raisedDegrees, kAutonomousLiftVelocityRpm);
	const std::int32_t rightResult =
	    gWinchRight->moveAbsolute(gWinchRightStartingPosition + raisedDegrees, kAutonomousLiftVelocityRpm);
	if (leftResult < 0 || rightResult < 0) {
		gWinchLeft->writeVoltage(0);
		gWinchRight->writeVoltage(0);
		std::printf("[autonomous] lift move failed: left=%ld right=%ld\n", static_cast<long>(leftResult),
		            static_cast<long>(rightResult));
		return false;
	}
	return true;
}

using MotionTestAction = void (*)(lightspeed::auton::AutonomousContext&, const lightspeed::odom::Pose&);

void runMotionTest(lightspeed::auton::AutonomousContext& ctx, const char* name, MotionTestAction action) {
	const lightspeed::odom::Pose start = ctx.odometry.getPose();
	std::printf("[%s] start pose=(%.2f, %.2f, %.1fdeg)\n", name, start.xInches, start.yInches, start.headingDegrees);
	action(ctx, start);
	const lightspeed::odom::Pose finish = ctx.odometry.getPose();
	std::printf("[%s] finish pose=(%.2f, %.2f, %.1fdeg), delta=(%.2f, %.2f)in, confidence=%s\n", name,
	            finish.xInches, finish.yInches, finish.headingDegrees, finish.xInches - start.xInches,
	            finish.yInches - start.yInches, lightspeed::odom::toString(ctx.odometry.getConfidence()));
}

// Gearing check: compares how far odometry (tracking wheel) says the robot
// went against how far the drive motors turned, giving the real inches of
// travel per motor revolution vs what kinematics is configured to assume. If
// they disagree, every commanded speed is off by that ratio.
void printDriveGearingCheck(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start,
                            double leftStartDeg, double rightStartDeg) {
	constexpr double kPi = 3.14159265358979323846;
	const lightspeed::odom::Pose finish = ctx.odometry.getPose();
	const double odomInches = std::hypot(finish.xInches - start.xInches, finish.yInches - start.yInches);
	const double motorRevs = (std::abs(gLeftDrive->getPositionRaw() - leftStartDeg) +
	                          std::abs(gRightDrive->getPositionRaw() - rightStartDeg)) /
	                         2.0 / 360.0;
	const auto& kin = lightspeed::motion::kCherenkovKinematics;
	const double configuredInchesPerRev = kPi * kin.wheelDiameterInches * kin.gearRatio;
	if (motorRevs > 0.01) {
		const double actualInchesPerRev = odomInches / motorRevs;
		std::printf("[gearingCheck] odom=%.2fin motorRevs=%.3f actual=%.3fin/rev configured=%.3fin/rev ratio=%.3f\n",
		            odomInches, motorRevs, actualInchesPerRev, configuredInchesPerRev,
		            actualInchesPerRev / configuredInchesPerRev);
	}
}

void driveStraightNoHold(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	const double leftStartDeg = gLeftDrive->getPositionRaw();
	const double rightStartDeg = gRightDrive->getPositionRaw();
	ctx.driveStraightDistance.run(kMotionTestDistanceInches, false);
	printDriveGearingCheck(ctx, start, leftStartDeg, rightStartDeg);
}

void driveStraightWithHold(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	const double leftStartDeg = gLeftDrive->getPositionRaw();
	const double rightStartDeg = gRightDrive->getPositionRaw();
	ctx.driveStraightDistance.run(kMotionTestDistanceInches);
	printDriveGearingCheck(ctx, start, leftStartDeg, rightStartDeg);
}

void turnToHeadingTest(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	ctx.turnToHeading.run(start.headingDegrees + 90.0);
}

void turnToPointTest(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	constexpr double kPi = 3.14159265358979323846;
	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double targetX = start.xInches + std::sin(headingRadians) * 12.0 + std::cos(headingRadians) * 12.0;
	const double targetY = start.yInches + std::cos(headingRadians) * 12.0 - std::sin(headingRadians) * 12.0;
	ctx.driveToPoint.turnToPoint(targetX, targetY);
}

void driveToPointTest(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	constexpr double kPi = 3.14159265358979323846;
	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double targetX = start.xInches + std::sin(headingRadians) * 12.0 + std::cos(headingRadians) * 6.0;
	const double targetY = start.yInches + std::cos(headingRadians) * 12.0 - std::sin(headingRadians) * 6.0;
	ctx.driveToPoint.driveToPoint(targetX, targetY);
}

void moveToPoseTest(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	constexpr double kPi = 3.14159265358979323846;
	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double targetX = start.xInches + std::sin(headingRadians) * 12.0;
	const double targetY = start.yInches + std::cos(headingRadians) * 12.0;
	ctx.moveToPose.run(targetX, targetY, start.headingDegrees + 90.0);
}

void purePursuitTest(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose& start) {
	constexpr double kPi = 3.14159265358979323846;
	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double forwardX = std::sin(headingRadians) * kMotionTestDistanceInches;
	const double forwardY = std::cos(headingRadians) * kMotionTestDistanceInches;
	const double rightX = std::cos(headingRadians) * 6.0;
	const double rightY = -std::sin(headingRadians) * 6.0;
	const std::vector<lightspeed::motion::Waypoint> path{
	    {start.xInches, start.yInches},
	    {start.xInches + forwardX, start.yInches + forwardY},
	    {start.xInches + forwardX + rightX, start.yInches + forwardY + rightY},
	};
	ctx.purePursuit.follow(path);
}

// Blocks until both winch motors are within tolerance of the lift target, or
// the timeout elapses. Returns true if the lift reached its target.
bool waitForLiftRaisedDegrees(double raisedDegrees, std::uint32_t timeoutMs,
                              double toleranceDegrees = kWinchPositionToleranceDegrees) {
	const std::uint32_t startTime = pros::millis();
	while (pros::millis() - startTime < timeoutMs) {
		const double leftRaised = gWinchLeftStartingPosition - gWinchLeft->getPositionRaw();
		const double rightRaised = gWinchRight->getPositionRaw() - gWinchRightStartingPosition;
		if (std::abs(leftRaised - raisedDegrees) <= toleranceDegrees &&
		    std::abs(rightRaised - raisedDegrees) <= toleranceDegrees) {
			return true;
		}
		pros::delay(kLoopPeriodMs);
	}
	return false;
}

void runCloseAlliance(lightspeed::auton::AutonomousContext& ctx) {
	constexpr double kPi = 3.14159265358979323846;
	const lightspeed::odom::Pose start = ctx.odometry.getPose();

	// -- Flip the toggle --
	// These are deliberate bumps: they need to arrive with speed, so they use
	// a sharper profile and skip the overshoot guard (which would otherwise
	// brake a 2in move to a crawl).
	ctx.drivetrain.setMaxOutputVoltageMillivolts(kCloseAllianceDriveVoltageLimitMillivolts);
	ctx.driveStraightDistance.run(kCloseAllianceToggleFirstPushInches, false, kCloseAllianceToggleProfile, false,
	                              kCloseAllianceToggleTimeoutSeconds);
	ctx.driveStraightDistance.run(-kCloseAllianceToggleBackUpInches, false, kCloseAllianceToggleProfile, false,
	                              kCloseAllianceToggleTimeoutSeconds);
	pros::delay(kCloseAllianceToggleReseatMs);  // let the toggle spring back before the second flip
	ctx.driveStraightDistance.run(kCloseAllianceToggleSecondPushInches, false, kCloseAllianceToggleProfile, false,
	                              kCloseAllianceToggleTimeoutSeconds);
	ctx.drivetrain.setMaxOutputVoltageMillivolts(12000);

	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double forwardX = std::sin(headingRadians);
	const double forwardY = std::cos(headingRadians);
	const double rightX = std::cos(headingRadians);
	const double rightY = -std::sin(headingRadians);
	const auto toField = [&](double backInches, double leftInches) {
		return lightspeed::motion::Waypoint{
		    start.xInches - forwardX * backInches - rightX * leftInches,
		    start.yInches - forwardY * backInches - rightY * leftInches,
		};
	};
	const lightspeed::motion::Waypoint target = toField(kCloseAllianceGoalBackInches, kCloseAllianceGoalLeftInches);

	// -- Raise the lift + extend the hinge, then drive there in one reversing
	// motion while the lift moves. moveAbsolute is non-blocking, so the lift
	// runs concurrently with the blocking path follow below. --
	const bool liftCommanded = commandLiftRaisedByEncoderDegrees(kAutonomousLiftRaiseDegrees);
	gHingePneumaticExtended = true;
	gHingePneumatic->set_value(gHingePneumaticExtended);

	const lightspeed::odom::Pose current = ctx.odometry.getPose();
	const double dx = current.xInches - start.xInches;
	const double dy = current.yInches - start.yInches;
	const double currentBack = -(dx * forwardX + dy * forwardY);
	const double currentLeft = -(dx * rightX + dy * rightY);

	// The quarter arc can't be bigger than the room available, or the path has
	// to go forward first. Shrink it to fit if needed.
	const double radius = std::max(
	    1.0, std::min({kCloseAllianceTurnRadiusInches, kCloseAllianceGoalBackInches - currentBack,
	                   kCloseAllianceGoalLeftInches - currentLeft - kCloseAllianceMinFinalStraightInches}));
	if (radius < kCloseAllianceTurnRadiusInches) {
		std::printf("[closeAlliance] arc radius reduced %.1f -> %.1fin to fit from (back %.2f, left %.2f)\n",
		            kCloseAllianceTurnRadiusInches, radius, currentBack, currentLeft);
	}
	const double arcStartBack = kCloseAllianceGoalBackInches - radius;

	std::vector<lightspeed::motion::Waypoint> goalPath{{current.xInches, current.yInches}};
	if (arcStartBack - currentBack > 0.5) {
		goalPath.push_back(toField(arcStartBack, currentLeft));  // end of straight reverse
	}
	// Explicit arc points, so the shape doesn't depend on path smoothing.
	constexpr int kArcSegments = 6;
	for (int i = 1; i <= kArcSegments; ++i) {
		const double theta = (kPi / 2.0) * i / kArcSegments;
		goalPath.push_back(toField(arcStartBack + radius * std::sin(theta), currentLeft + radius * (1.0 - std::cos(theta))));
	}
	goalPath.push_back(target);  // straight reverse into goal

	ctx.purePursuit.follow(goalPath, true, 0, kCloseAllianceGoalLookaheadInches, kCloseAllianceGoalToleranceInches,
	                       kCloseAllianceGoalTimeoutSeconds);
	ctx.driveStraightDistance.run(-kGoalSeatPushInches, true, lightspeed::motion::kDriveStraightDistanceConfig.motionProfile,
	                              false, kGoalSeatPushTimeoutSeconds);

	// -- Score: quick, loose check that the lift is up before releasing the
	// pin. It has had the whole drive to get there, so this rarely waits. --
	if (liftCommanded && !waitForLiftRaisedDegrees(kAutonomousLiftRaiseDegrees, kCloseAllianceScoreLiftWaitTimeoutMs,
	                                               kCloseAllianceScoreLiftToleranceDegrees)) {
		std::printf("[closeAlliance] lift did not reach %.0fdeg before timeout, scoring anyway\n",
		            kAutonomousLiftRaiseDegrees);
	}
	gClawPneumaticExtended = true;
	gClawPneumatic->set_value(gClawPneumaticExtended);

	const lightspeed::odom::Pose scored = ctx.odometry.getPose();
	std::printf("[closeAlliance] goal target=(%.2f, %.2f, %.1fdeg) reached=(%.2f, %.2f, %.1fdeg)\n", target.x, target.y,
	            std::fmod(start.headingDegrees + 90.0, 360.0), scored.xInches, scored.yInches, scored.headingDegrees);

	// -- Pull forward off the goal. Give the claw time to release the pin
	// first so driving away doesn't drag it off. The second lift raise starts
	// with the drive and keeps going through the turn below. --
	pros::delay(kCloseAllianceClawReleaseMs);
	const double secondLiftDegrees = kAutonomousLiftRaiseDegrees + kCloseAllianceSecondLiftRaiseDegrees;
	const bool secondLiftCommanded = commandLiftRaisedByEncoderDegrees(secondLiftDegrees);
	const lightspeed::motion::Waypoint secondTarget =
	    toField(kCloseAllianceSecondBackInches, kCloseAllianceSecondLeftInches);
	ctx.purePursuit.follow({{scored.xInches, scored.yInches}, secondTarget});

	const lightspeed::odom::Pose finish = ctx.odometry.getPose();
	std::printf("[closeAlliance] second target=(%.2f, %.2f) reached=(%.2f, %.2f, %.1fdeg)\n", secondTarget.x,
	            secondTarget.y, finish.xInches, finish.yInches, finish.headingDegrees);

	// -- Turn to start + kCloseAllianceTurnRelativeDegrees (clockwise-positive)
	// while the lift finishes. --
	const double turnTargetDegrees = std::fmod(start.headingDegrees + kCloseAllianceTurnRelativeDegrees, 360.0);
	ctx.turnToHeading.run(turnTargetDegrees);
	if (secondLiftCommanded && !waitForLiftRaisedDegrees(secondLiftDegrees, kCloseAllianceLiftWaitTimeoutMs)) {
		std::printf("[closeAlliance] lift did not reach %.0fdeg before timeout\n", secondLiftDegrees);
	}

	const lightspeed::odom::Pose turned = ctx.odometry.getPose();
	std::printf("[closeAlliance] turn target=%.1fdeg reached=%.1fdeg\n", turnTargetDegrees, turned.headingDegrees);

	// -- Straight reverse, holding the post-turn heading --
	ctx.driveStraightDistance.run(-kCloseAllianceFinalReverseInches);

	const lightspeed::odom::Pose reversed = ctx.odometry.getPose();
	std::printf("[closeAlliance] final reverse %.2fin reached=(%.2f, %.2f, %.1fdeg)\n", kCloseAllianceFinalReverseInches,
	            reversed.xInches, reversed.yInches, reversed.headingDegrees);

	// ===== Cup + pin pairs: pick up, back into the goal, score. =====
	// Claw: false = closed (grab), true = open (release).
	const auto setClaw = [](bool open) {
		gClawPneumaticExtended = open;
		gClawPneumatic->set_value(gClawPneumaticExtended);
	};
	const auto moveLiftAndWait = [](double raisedDegrees, const char* step) {
		if (commandLiftRaisedByEncoderDegrees(raisedDegrees) &&
		    !waitForLiftRaisedDegrees(raisedDegrees, kPairLiftWaitTimeoutMs, kPairLiftToleranceDegrees)) {
			std::printf("[closeAlliance] %s: lift did not reach %.0fdeg before timeout, continuing\n", step, raisedDegrees);
		}
	};
	// Turn so the BACK faces the point, then reverse straight toward it,
	// stopping stopShortInches before it. Uses the overshoot guard, so it won't
	// drive past a stack. timeoutSeconds > 0 limits the move (for approaches
	// that end pressed against something). shiftRightInches is a field trim:
	// it moves the aim point to the robot's right as it will be facing while
	// backing in (negative = left).
	const auto reverseToPoint = [&](const lightspeed::motion::Waypoint& target, double stopShortInches,
	                                double shiftRightInches, double timeoutSeconds, const char* step) {
		const lightspeed::odom::Pose pose = ctx.odometry.getPose();
		const double approachHeadingRadians =
		    std::atan2(target.x - pose.xInches, target.y - pose.yInches) + kPi;  // back faces the target
		const lightspeed::motion::Waypoint point{
		    target.x + std::cos(approachHeadingRadians) * shiftRightInches,
		    target.y - std::sin(approachHeadingRadians) * shiftRightInches,
		};
		const double toX = point.x - pose.xInches;
		const double toY = point.y - pose.yInches;
		const double distanceInches = std::hypot(toX, toY) - stopShortInches;
		if (distanceInches < 0.5) {
			return;
		}
		const double bearingDegrees = std::atan2(toX, toY) * 180.0 / kPi;  // clockwise from +y, like headings
		ctx.turnToHeading.run(std::fmod(bearingDegrees + 180.0 + 360.0, 360.0));
		ctx.driveStraightDistance.run(-distanceInches, true,
		                              lightspeed::motion::kDriveStraightDistanceConfig.motionProfile, true,
		                              timeoutSeconds);
		const lightspeed::odom::Pose arrived = ctx.odometry.getPose();
		std::printf("[closeAlliance] %s target=(%.2f, %.2f) reached=(%.2f, %.2f, %.1fdeg)\n", step, point.x, point.y,
		            arrived.xInches, arrived.yInches, arrived.headingDegrees);
	};
	// Straight from wherever the pair was picked up: back faces the goal's
	// center, then reverse in to contact distance (or until the goal stops it /
	// kPairGoalTimeoutSeconds). Goal center = where the robot sat scoring the
	// pin, plus the contact distance in the direction it was backing then.
	const auto backIntoGoal = [&](double extraBackInches, double shiftRightInches, const char* step) {
		const lightspeed::motion::Waypoint goalCenter =
		    toField(kCloseAllianceGoalBackInches, kCloseAllianceGoalLeftInches + kGoalContactDistanceInches);
		reverseToPoint(goalCenter, kGoalContactDistanceInches - extraBackInches, shiftRightInches, kPairGoalTimeoutSeconds,
		               step);
	};
	const auto pickUpPair = [&](const char* step) {
		moveLiftAndWait(kPairPickupLiftDegrees, step);
		setClaw(false);
		pros::delay(kPairGrabSettleMs);
		moveLiftAndWait(kPairScoreLiftDegrees, step);
	};
	const auto scorePair = [&](const char* step) {
		setClaw(true);
		pros::delay(kCloseAllianceClawReleaseMs);
		moveLiftAndWait(kPairClearLiftDegrees, step);
	};

	// -- Pair 1: the robot is already at its pickup position. --
	pickUpPair("pair1 pickup");
	backIntoGoal(kPairGoalExtraBackInches, kPairGoalShiftRightInches, "pair1 to goal");
	scorePair("pair1 score");

	// -- Pair 2 --
	if (!kSecondPairPositionSet) {
		std::printf("[closeAlliance] second pair position not set (kSecondPairPositionSet) -- stopping after pair 1\n");
		return;
	}
	// Drive forward off the goal (the robot's back is on it), then turn so the
	// BACK -- where the claw is -- faces the pickup point and reverse to it.
	// (Driving forward all the way would arrive with the claw facing away.)
	ctx.driveStraightDistance.run(kPairGoalPullOffInches);
	reverseToPoint(toField(kPair2StagingBackInches, kSecondPairLeftInches), 0.0, 0.0, 0.0, "pair2 to staging");
	reverseToPoint(toField(kSecondPairBackInches, kSecondPairLeftInches), 0.0, kPair2PickupShiftRightInches, 0.0,
	               "pair2 to pickup");
	pickUpPair("pair2 pickup");
	backIntoGoal(kPair2GoalExtraBackInches, kPair2GoalShiftRightInches, "pair2 to goal");
	scorePair("pair2 score");
}

}  // namespace

// Builds every shared object above (IMU calibration blocks ~2-3s here),
// registers the demo routines, and starts the GUI selector so a driver can
// pick a start location and routine any time before the match.
void initialize() {
	using namespace lightspeed;

	gLeftDrive.emplace(hal::config::kLeftDriveGroup);
	gRightDrive.emplace(hal::config::kRightDriveGroup);
	gWinchLeft.emplace(hal::config::kWinchLeftGroup);
	gWinchRight.emplace(hal::config::kWinchRightGroup);
	gIntake.emplace(hal::config::kIntakeGroup);
	gClawPneumatic.emplace(hal::config::adi_port::kClawPneumatic);
	gHingePneumatic.emplace(hal::config::adi_port::kHingePneumatic);
	gLiftMinimumLimitSwitch.emplace(hal::config::adi_port::kLiftMinimumLimitSwitch);
	gCalibratedIndicator.emplace(hal::config::adi_port::kCalibratedIndicator);
	gCalibrationIndicator.emplace(hal::config::adi_port::kCalibrationIndicator);
	gWinchLeft->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	gWinchRight->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	gWinchLeftStartingPosition = gWinchLeft->getPositionRaw();
	gWinchRightStartingPosition = gWinchRight->getPositionRaw();
	gDrivetrain.emplace(*gLeftDrive, *gRightDrive, control::kDrivetrainVelocityConfig);

	gPrimaryImu.emplace(hal::config::kPrimaryImu.port);
	gSecondaryImu.emplace(hal::config::kSecondaryImu.port);
	gCalibratedIndicator->set_value(false);
	gCalibrationIndicator->set_value(true);
	gPrimaryImu->calibrate(true);
	gSecondaryImu->calibrate(true);
	if (gPrimaryImu->isReady() && gSecondaryImu->isReady()) {
		gCalibrationIndicator->set_value(false);
		gCalibratedIndicator->set_value(true);
	}
	gOdometryPodRotation.emplace(hal::config::kOdometryPodRotation.port);

	gLeftIme.emplace(*gLeftDrive, odom::kDriveImeConfig);
	gRightIme.emplace(*gRightDrive, odom::kDriveImeConfig);
	gImuSource.emplace(*gPrimaryImu, &*gSecondaryImu);
	gOdometryPod.emplace(*gOdometryPodRotation, odom::kCherenkovTopology.pods[0]);

	std::vector<odom::TrackingWheelSource*> pods{&*gOdometryPod};
	gOdometry.emplace(odom::kCherenkovTopology.kinematics, *gLeftIme, *gRightIme, *gImuSource, pods);

	gTurnToHeading.emplace(*gDrivetrain, *gOdometry, motion::kTurnToHeadingConfig);
	gDriveStraightDistance.emplace(*gDrivetrain, *gOdometry, motion::kDriveStraightDistanceConfig);
	gDriveToPoint.emplace(*gTurnToHeading, *gDriveStraightDistance, *gOdometry);
	gMoveToPose.emplace(*gDrivetrain, *gOdometry, motion::kMoveToPoseConfig);
	gPurePursuit.emplace(*gDrivetrain, *gOdometry, motion::kPurePursuitConfig);

	gInputProfile.emplace(driver::kInputProfileConfig);
	gAccelLimitResolver.emplace(driver::kDriveAccelLimitConfig);
	gLeftAccelSlew.emplace(driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond);
	gRightAccelSlew.emplace(driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond);

	gRoutineRegistry.registerRoutine(auton::demo::makeDemoStraightAndTurnRoutine());
	gRoutineRegistry.registerRoutine(auton::demo::makeDemoPursuitPathRoutine());
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Straight 12in, No Hold",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {12.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "straightNoHold", driveStraightNoHold); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Straight 12in + Hold",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {12.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "straightWithHold", driveStraightWithHold); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Turn +90deg",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "turnToHeading", turnToHeadingTest); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Turn To Point",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {24.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "turnToPoint", turnToPointTest); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Drive To Point",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {18.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "driveToPoint", driveToPointTest); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Move To Pose",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {12.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "moveToPose", moveToPoseTest); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "Test: Pure Pursuit",
	    .validStartLocationIndices = kMotionTestStartLocationIndices,
	    .previewPoints = {{12.0, 12.0}, {12.0, 24.0}, {18.0, 24.0}},
	    .run = [](auton::AutonomousContext& ctx) { runMotionTest(ctx, "purePursuit", purePursuitTest); },
	});
	gRoutineRegistry.registerRoutine(auton::Routine{
	    .name = "closeAlliance",
	    .validStartLocationIndices = {auton::kCloseAllianceStartLocationIndex},
	    .previewPoints = {{0.0, 11.375}, {0.0, 9.375}, {0.0, 10.875}, {0.0, 8.875}, {18.0, 21.375}, {9.0, 31.0}},
	    .run = runCloseAlliance,
	});

	gSelectorGui.emplace(*gOdometry, gRoutineRegistry);

	gSerialLink.emplace(telemetry::TelemetryBus::instance());

	// No-op unless Y is held at startup, in which case it starts serial
	// telemetry streaming and returns; normal operation continues.
	diagnostics::runDiagnosticModeIfRequested(*gSerialLink);

	gSelectorGui->start();

	// Both gate themselves on competition state internally, so starting them
	// here is safe.
	gSdLogger.emplace(telemetry::TelemetryBus::instance());
	gSdLogger->start();
	gDashboard.emplace(*gOdometry, *gDrivetrain, gLimpModeEnabled);
	gDashboard->start();
}

void disabled() {}

void competition_initialize() {}

// Releases the screen from the GUI selector (only one task may draw at a
// time) and runs whichever routine getRoutineForAutonomous() resolves to.
void autonomous() {
	using namespace lightspeed;

	gLeftDrive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	gRightDrive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
	gSelectorGui->stop();

	bool usedFallback = false;
	const auton::Routine* routine = gSelectorGui->getRoutineForAutonomous(&usedFallback);
	if (routine == nullptr) {
		telemetry::TelemetryBus::instance().record("auton.routineName", "none");
		std::printf("[autonomous] no routine confirmed or previewed via the GUI selector -- nothing to run\n");
		return;
	}

	telemetry::TelemetryBus::instance().record("auton.routineName", routine->name);
	if (usedFallback) {
		std::printf(
		    "[autonomous] WARNING: no routine was confirmed -- running the last previewed routine as a fallback: %s\n",
		    routine->name);
	} else {
		std::printf("[autonomous] running confirmed routine: %s\n", routine->name);
	}

	auton::AutonomousContext ctx{
	    .drivetrain = *gDrivetrain,
	    .odometry = *gOdometry,
	    .turnToHeading = *gTurnToHeading,
	    .driveStraightDistance = *gDriveStraightDistance,
	    .driveToPoint = *gDriveToPoint,
	    .purePursuit = *gPurePursuit,
	    .moveToPose = *gMoveToPose,
	};
	routine->run(ctx);

	const odom::Pose finalPose = gOdometry->getPose();
	std::printf("[autonomous] routine complete: final pose=(%.2f, %.2f, %.1fdeg)\n", finalPose.xInches, finalPose.yInches,
	            finalPose.headingDegrees);
}

// Driver-control loop: input profiling -> drive-mode -> accel-limited-slew
// -> velocity-controller.
void opcontrol() {
	using namespace lightspeed;

	gLeftDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	gRightDrive->setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

	// Idempotent -- autonomous() already stops the selector in the normal
	// flow. This covers bench testing where autonomous() never runs, so the
	// selector and dashboard tasks never both hold pros::screen.
	gSelectorGui->stop();

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	std::int32_t liftMinimumSwitchValue = gLiftMinimumLimitSwitch->get_value();
	bool previousLiftMinimumActive = liftMinimumSwitchValue == 1;
	bool liftMinimumSwitchStateKnown = liftMinimumSwitchValue == 0 || liftMinimumSwitchValue == 1;
	bool liftMinimumSwitchErrorReported = false;

	gLimpModeEnabled.store(false);
	gLeftAccelSlew->reset();
	gRightAccelSlew->reset();
	gDrivetrain->setTargetVelocity(0.0, 0.0);
	gDrivetrain->coast();

	double previousAccelLimit = driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond;
	std::uint32_t previousStatusTime = pros::millis();

	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
			const bool limpModeEnabled = !gLimpModeEnabled.load();
			gLimpModeEnabled.store(limpModeEnabled);
			gLeftAccelSlew->reset();
			gRightAccelSlew->reset();
			if (limpModeEnabled) {
				gDrivetrain->setTargetVelocity(0.0, 0.0);
				gDrivetrain->coast();
			}
			std::printf("[opcontrol] limp mode %s\n", limpModeEnabled ? "enabled" : "disabled");
		}

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			gClawPneumaticExtended = !gClawPneumaticExtended;
			gClawPneumatic->set_value(gClawPneumaticExtended);
		}
		liftMinimumSwitchValue = gLiftMinimumLimitSwitch->get_value();
		if (liftMinimumSwitchValue == 0 || liftMinimumSwitchValue == 1) {
			const bool liftMinimumActive = liftMinimumSwitchValue == 1;
			if (liftMinimumSwitchStateKnown && previousLiftMinimumActive && !liftMinimumActive) {
				gClawPneumaticExtended = false;
				gClawPneumatic->set_value(gClawPneumaticExtended);
			}
			previousLiftMinimumActive = liftMinimumActive;
			liftMinimumSwitchStateKnown = true;
			liftMinimumSwitchErrorReported = false;
		} else if (!liftMinimumSwitchErrorReported) {
			std::printf("[opcontrol] lift minimum limit switch read failed: %ld\n",
			            static_cast<long>(liftMinimumSwitchValue));
			liftMinimumSwitchErrorReported = true;
		}
		const double liftRaisedDegrees =
		    ((gWinchLeftStartingPosition - gWinchLeft->getPositionRaw()) +
		     (gWinchRight->getPositionRaw() - gWinchRightStartingPosition)) /
		    2.0;
		const bool shouldExtendHinge = liftRaisedDegrees > kHingeAutoExtendLiftDegrees;
		if (gHingePneumaticExtended != shouldExtendHinge) {
			gHingePneumaticExtended = shouldExtendHinge;
			gHingePneumatic->set_value(gHingePneumaticExtended);
		}

		// -- Input profiling --
		const driver::JoystickInput rawInput{
		    .leftY = normalizeStick(master.get_analog(ANALOG_LEFT_Y)),
		    .leftX = normalizeStick(master.get_analog(ANALOG_LEFT_X)),
		    .rightY = normalizeStick(master.get_analog(ANALOG_RIGHT_Y)),
		    .rightX = normalizeStick(master.get_analog(ANALOG_RIGHT_X)),
		};
		const driver::JoystickInput profiledInput{
		    .leftY = gInputProfile->apply(rawInput.leftY),
		    .leftX = gInputProfile->apply(rawInput.leftX),
		    .rightY = gInputProfile->apply(rawInput.rightY),
		    .rightX = gInputProfile->apply(rawInput.rightX),
		};

		// -- Drive-mode transform --
		const driver::DriveOutput driveOutput = driver::computeDriveOutput(driver::kDriveMode, profiledInput);
		const double leftTargetRpm = driveOutput.left * driver::kMaxDriveRpm;
		const double rightTargetRpm = driveOutput.right * driver::kMaxDriveRpm;

		// -- Condition-driven accel limit, applied to the slew limiter --
		const double accelLimit = gAccelLimitResolver->resolve();
		gLeftAccelSlew->setMaxRate(accelLimit);
		gRightAccelSlew->setMaxRate(accelLimit);
		double leftSlewedRpm = 0.0;
		double rightSlewedRpm = 0.0;
		if (gLimpModeEnabled.load()) {
			gLeftAccelSlew->reset();
			gRightAccelSlew->reset();
			gDrivetrain->coast();
		} else if (leftTargetRpm == 0.0 && rightTargetRpm == 0.0) {
			gLeftAccelSlew->reset();
			gRightAccelSlew->reset();
			gDrivetrain->coast();
		} else {
			leftSlewedRpm = gLeftAccelSlew->calculate(leftTargetRpm, kDtSeconds);
			rightSlewedRpm = gRightAccelSlew->calculate(rightTargetRpm, kDtSeconds);
			gDrivetrain->setTargetVelocity(leftSlewedRpm, rightSlewedRpm);
		}

		const bool winchWindRequested = master.get_digital(DIGITAL_L1);
		const bool winchLetOutRequested = !winchWindRequested && master.get_digital(DIGITAL_L2);

		if (!winchWindRequested) {
			gWinchStoppedForNoLoad = false;
			gWinchLowCurrentSince.reset();
		} else if (!gWinchStoppedForNoLoad) {
			const auto leftCurrent = gWinchLeft->getCurrentDrawMilliAmps();
			const auto rightCurrent = gWinchRight->getCurrentDrawMilliAmps();
			if (!leftCurrent || !rightCurrent) {
				gWinchStoppedForNoLoad = true;
				gWinchLowCurrentSince.reset();
				std::printf("[opcontrol] winch stopped: unable to read motor current\n");
			} else if (*leftCurrent <= kWinchNoLoadCurrentMilliAmps ||
			           *rightCurrent <= kWinchNoLoadCurrentMilliAmps) {
				const std::uint32_t now = pros::millis();
				if (!gWinchLowCurrentSince) {
					gWinchLowCurrentSince = now;
				} else if (now - *gWinchLowCurrentSince >= kWinchNoLoadConfirmMs) {
					gWinchStoppedForNoLoad = true;
					std::printf("[opcontrol] winch stopped: no-load current detected\n");
				}
			} else {
				gWinchLowCurrentSince.reset();
			}
		}

		if (winchWindRequested && !gWinchStoppedForNoLoad) {
			gWinchLeft->writeVoltage(-kWinchVoltage);
			gWinchRight->writeVoltage(kWinchVoltage);
		} else if (winchLetOutRequested &&
		           gWinchLeft->getPositionRaw() <
		               gWinchLeftStartingPosition - kWinchPositionToleranceDegrees &&
		           gWinchRight->getPositionRaw() >
		               gWinchRightStartingPosition + kWinchPositionToleranceDegrees) {
			const std::int32_t loweringVoltage =
			    liftRaisedDegrees <= kHingeAutoExtendLiftDegrees
			        ? static_cast<std::int32_t>(kWinchVoltage * kWinchLoweringVoltageScaleBelowHingeThreshold)
			        : kWinchVoltage;
			gWinchLeft->writeVoltage(loweringVoltage);
			gWinchRight->writeVoltage(-loweringVoltage);
		} else {
			gWinchLeft->writeVoltage(0);
			gWinchRight->writeVoltage(0);
		}

		if (master.get_digital(DIGITAL_B)) {
			gIntake->writeVoltage(kIntakeVoltage);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			gIntake->writeVoltage(-kIntakeVoltage);
		} else {
			gIntake->writeVoltage(0);
		}

		if (accelLimit != previousAccelLimit) {
			std::printf("[opcontrol] accel limit changed: %.0f -> %.0f RPM/s\n", previousAccelLimit, accelLimit);
			previousAccelLimit = accelLimit;
		}

		const std::uint32_t now = pros::millis();
		if (now - previousStatusTime >= kStatusIntervalMs) {
			const odom::Pose statusPose = gOdometry->getPose();
			std::printf("[opcontrol] target=(%.0f, %.0f)rpm slewed=(%.0f, %.0f)rpm accelLimit=%.0fRPM/s lift=%.0fdeg "
			            "pose=(%.2f, %.2f, %.1fdeg)\n",
			            leftTargetRpm, rightTargetRpm, leftSlewedRpm, rightSlewedRpm, accelLimit, liftRaisedDegrees,
			            statusPose.xInches, statusPose.yInches, statusPose.headingDegrees);
			previousStatusTime = now;
		}

		pros::delay(kLoopPeriodMs);
	}
}
