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
constexpr double kAutonomousLiftRaiseDegrees = 100.0;
constexpr std::int32_t kAutonomousLiftVelocityRpm = 100;
const std::vector<std::size_t> kMotionTestStartLocationIndices{0, 1};

double normalizeStick(std::int32_t rawAnalog) {
	return static_cast<double>(rawAnalog) / 127.0;
}

bool commandLiftRaisedByEncoderDegrees(double raisedDegrees) {
	const double leftStartPosition = gWinchLeft->getPositionRaw();
	const double rightStartPosition = gWinchRight->getPositionRaw();
	const std::int32_t leftResult =
	    gWinchLeft->moveAbsolute(leftStartPosition - raisedDegrees, kAutonomousLiftVelocityRpm);
	const std::int32_t rightResult =
	    gWinchRight->moveAbsolute(rightStartPosition + raisedDegrees, kAutonomousLiftVelocityRpm);
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

void driveStraightNoHold(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose&) {
	ctx.driveStraightDistance.run(kMotionTestDistanceInches, false);
}

void driveStraightWithHold(lightspeed::auton::AutonomousContext& ctx, const lightspeed::odom::Pose&) {
	ctx.driveStraightDistance.run(kMotionTestDistanceInches);
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

void runCloseAlliance(lightspeed::auton::AutonomousContext& ctx) {
	ctx.drivetrain.setMaxOutputVoltageMillivolts(kCloseAllianceDriveVoltageLimitMillivolts);
	ctx.driveStraightDistance.run(2.0, false);
	ctx.driveStraightDistance.run(-1.75, false);
	ctx.driveStraightDistance.run(2.5, false);
	ctx.drivetrain.setMaxOutputVoltageMillivolts(12000);

	constexpr double kTargetX = 19.0;
	constexpr double kTargetY = 18.5;
	constexpr lightspeed::motion::MotionProfileConfig kBackwardMoveProfile{
	    .maxVelocity = 28.0,
	    .maxAcceleration = 40.0,
	    .maxJerk = 0.0,
	};
	const lightspeed::odom::Pose start = ctx.odometry.getPose();
	const double targetHeading = lightspeed::motion::headingToPoint(start, kTargetX, kTargetY);
	const double backwardsHeading = targetHeading + 180.0;
	const double distance = lightspeed::motion::distanceToPoint(start, kTargetX, kTargetY);
	ctx.turnToHeading.run(backwardsHeading);
	if (!commandLiftRaisedByEncoderDegrees(kAutonomousLiftRaiseDegrees)) {
		return;
	}
	gHingePneumaticExtended = true;
	gHingePneumatic->set_value(gHingePneumaticExtended);
	ctx.driveStraightDistance.run(-distance, false, kBackwardMoveProfile);
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
	    .previewPoints = {{0.0, 11.375}, {0.0, 9.375}, {0.0, 10.875}, {0.0, 8.875}, {18.0, 18.0}},
	    .run = runCloseAlliance,
	});

	gSelectorGui.emplace(*gOdometry, gRoutineRegistry);

	gSerialLink.emplace(telemetry::TelemetryBus::instance());

	// No-op unless Y was held at boot, in which case it never returns and
	// nothing below starts this boot.
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
			std::printf("[opcontrol] target=(%.0f, %.0f)rpm slewed=(%.0f, %.0f)rpm accelLimit=%.0fRPM/s\n",
			            leftTargetRpm, rightTargetRpm, leftSlewedRpm, rightSlewedRpm, accelLimit);
			previousStatusTime = now;
		}

		pros::delay(kLoopPeriodMs);
	}
}
