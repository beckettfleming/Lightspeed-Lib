#include "main.h"

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
#include "lightspeed/subsystem/lift.hpp"
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
std::optional<lightspeed::subsystem::Lift> gLift;
std::optional<lightspeed::hal::MotorGroup> gIntake;
std::optional<pros::adi::DigitalOut> gClawPneumatic;
std::optional<pros::adi::DigitalOut> gHingePneumatic;
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
std::optional<lightspeed::hal::RotationSensor> gForwardPodRotation;
std::optional<lightspeed::hal::RotationSensor> gSidewaysPodRotation;
std::optional<lightspeed::odom::IMESource> gLeftIme;
std::optional<lightspeed::odom::IMESource> gRightIme;
std::optional<lightspeed::odom::IMUSource> gImuSource;
std::optional<lightspeed::odom::TrackingWheelSource> gForwardPod;
std::optional<lightspeed::odom::TrackingWheelSource> gSidewaysPod;
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
constexpr std::int32_t kWinchVoltage = 12000;
constexpr std::int32_t kIntakeVoltage = 12000;
constexpr double kWinchDrumRadiusInches = 0.5;
constexpr std::uint32_t kLiftSettleTimeoutMs = 5000;
constexpr double kMotionTestSegmentInches = 6.0;

double normalizeStick(std::int32_t rawAnalog) {
	return static_cast<double>(rawAnalog) / 127.0;
}

void runSelectorMotionTest(lightspeed::auton::AutonomousContext& ctx) {
	constexpr double kPi = 3.14159265358979323846;
	const lightspeed::odom::Pose start = ctx.odometry.getPose();
	const double headingRadians = start.headingDegrees * kPi / 180.0;
	const double forwardX = std::sin(headingRadians) * kMotionTestSegmentInches;
	const double forwardY = std::cos(headingRadians) * kMotionTestSegmentInches;
	const double rightX = std::cos(headingRadians) * kMotionTestSegmentInches;
	const double rightY = -std::sin(headingRadians) * kMotionTestSegmentInches;

	const double forwardTargetX = start.xInches + forwardX;
	const double forwardTargetY = start.yInches + forwardY;
	const double sideTargetX = forwardTargetX + rightX;
	const double sideTargetY = forwardTargetY + rightY;
	const double finalTargetX = sideTargetX + rightX;
	const double finalTargetY = sideTargetY + rightY;
	const double sideHeading = start.headingDegrees + 90.0;

	ctx.driveStraightDistance.run(kMotionTestSegmentInches);
	ctx.turnToHeading.run(sideHeading);
	ctx.driveToPoint.driveToPoint(sideTargetX, sideTargetY);
	ctx.moveToPose.run(finalTargetX, finalTargetY, sideHeading);

	const std::vector<lightspeed::motion::Waypoint> returnPath{
	    {finalTargetX, finalTargetY},
	    {sideTargetX, sideTargetY},
	    {forwardTargetX, forwardTargetY},
	    {start.xInches, start.yInches},
	};
	ctx.purePursuit.follow(returnPath);
}

void runCloseAlliance(lightspeed::auton::AutonomousContext& ctx) {
	ctx.turnToHeading.run(180.0);
	ctx.driveStraightDistance.run(2.0);
	ctx.driveStraightDistance.run(-1.5);
	ctx.driveStraightDistance.run(2.0);

	const lightspeed::odom::Pose pose = ctx.odometry.getPose();
	const double targetX = 18.0;
	const double targetY = 18.0;
	const double distanceToTarget = lightspeed::motion::distanceToPoint(pose, targetX, targetY);
	double reverseHeading = lightspeed::motion::headingToPoint(pose, targetX, targetY) + 180.0;
	if (reverseHeading >= 360.0) {
		reverseHeading -= 360.0;
	}

	ctx.turnToHeading.run(reverseHeading);
	gHingePneumaticExtended = true;
	gHingePneumatic->set_value(gHingePneumaticExtended);
	const bool liftMoveStarted = gLift->moveByInches(2.0);
	ctx.driveStraightDistance.run(-distanceToTarget);

	if (liftMoveStarted) {
		const std::uint32_t liftWaitStarted = pros::millis();
		while (!gLift->isAtTarget()) {
			if (pros::millis() - liftWaitStarted >= kLiftSettleTimeoutMs) {
				std::printf("[autonomous] WARNING: lift did not reach 2in target (measured %.2fin)\n",
				            gLift->getHeightInches());
				gLift->stop();
				break;
			}
			pros::delay(10);
		}
	}
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
	gCalibratedIndicator.emplace(hal::config::adi_port::kCalibratedIndicator);
	gCalibrationIndicator.emplace(hal::config::adi_port::kCalibrationIndicator);
	gWinchLeft->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	gWinchRight->setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
	gWinchLeftStartingPosition = gWinchLeft->getPositionRaw();
	gWinchRightStartingPosition = gWinchRight->getPositionRaw();
	gLift.emplace(*gWinchLeft, *gWinchRight, kWinchDrumRadiusInches);
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
	gForwardPodRotation.emplace(hal::config::kForwardPodRotation.port);
	gSidewaysPodRotation.emplace(hal::config::kSidewaysPodRotation.port);

	gLeftIme.emplace(*gLeftDrive, odom::kDriveImeConfig);
	gRightIme.emplace(*gRightDrive, odom::kDriveImeConfig);
	gImuSource.emplace(*gPrimaryImu, &*gSecondaryImu);
	gForwardPod.emplace(*gForwardPodRotation, odom::kCherenkovTopology.pods[0]);
	gSidewaysPod.emplace(*gSidewaysPodRotation, odom::kCherenkovTopology.pods[1]);

	std::vector<odom::TrackingWheelSource*> pods{&*gForwardPod, &*gSidewaysPod};
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
	    .name = "Test: Selector + Motion",
	    .validStartLocationIndices = {},
	    .previewPoints = {{12.0, 12.0}, {12.0, 18.0}, {18.0, 18.0}, {24.0, 18.0},
	                      {18.0, 18.0}, {12.0, 18.0}, {12.0, 12.0}},
	    .run = runSelectorMotionTest,
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
	gDashboard.emplace(*gOdometry);
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

	double previousAccelLimit = driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond;
	std::uint32_t previousStatusTime = pros::millis();

	while (true) {
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
			gClawPneumaticExtended = !gClawPneumaticExtended;
			gClawPneumatic->set_value(gClawPneumaticExtended);
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
			gHingePneumaticExtended = !gHingePneumaticExtended;
			gHingePneumatic->set_value(gHingePneumaticExtended);
		}

		// -- Input profiling --
		const driver::JoystickInput rawInput{
		    .leftY = -normalizeStick(master.get_analog(ANALOG_LEFT_Y)),
		    .leftX = normalizeStick(master.get_analog(ANALOG_LEFT_X)),
		    .rightY = normalizeStick(master.get_analog(ANALOG_RIGHT_Y)),
		    .rightX = -normalizeStick(master.get_analog(ANALOG_RIGHT_X)),
		};
		const driver::JoystickInput profiledInput{
		    .leftY = gInputProfile->apply(rawInput.rightX),
		    .leftX = gInputProfile->apply(rawInput.rightY),
		    .rightY = gInputProfile->apply(rawInput.leftX),
		    .rightX = gInputProfile->apply(rawInput.leftY),
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
		if (leftTargetRpm == 0.0 && rightTargetRpm == 0.0) {
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
			gWinchLeft->writeVoltage(kWinchVoltage);
			gWinchRight->writeVoltage(-kWinchVoltage);
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
