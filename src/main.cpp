#include "main.h"

#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

#include "lightspeed/auton/auton_constants.hpp"
#include "lightspeed/auton/autonomous_context.hpp"
#include "lightspeed/auton/demo_routines.hpp"
#include "lightspeed/auton/routine.hpp"
#include "lightspeed/auton/selector_gui.hpp"
#include "lightspeed/control/drivetrain_velocity_constants.hpp"
#include "lightspeed/control/drivetrain_velocity_controller.hpp"
#include "lightspeed/control/slew_rate_limiter.hpp"
#include "lightspeed/diagnostics/diagnostic_mode.hpp"
#include "lightspeed/driver/accel_limit_resolver.hpp"
#include "lightspeed/driver/button_macro.hpp"
#include "lightspeed/driver/demo_macros.hpp"
#include "lightspeed/driver/drive_mode.hpp"
#include "lightspeed/driver/driver_control_constants.hpp"
#include "lightspeed/driver/input_profile.hpp"
#include "lightspeed/hal/ai_vision_sensor.hpp"
#include "lightspeed/hal/config.hpp"
#include "lightspeed/hal/imu.hpp"
#include "lightspeed/hal/motor_group.hpp"
#include "lightspeed/motion/drive_straight_distance.hpp"
#include "lightspeed/motion/drive_to_point.hpp"
#include "lightspeed/motion/motion_constants.hpp"
#include "lightspeed/motion/move_to_pose.hpp"
#include "lightspeed/motion/pure_pursuit_controller.hpp"
#include "lightspeed/motion/turn_to_heading.hpp"
#include "lightspeed/odom/ime_source.hpp"
#include "lightspeed/odom/imu_source.hpp"
#include "lightspeed/odom/odometry_constants.hpp"
#include "lightspeed/odom/odometry_fusion.hpp"
#include "lightspeed/odom/tracking_wheel_source.hpp"
#include "lightspeed/odom/types.hpp"
#include "lightspeed/subsystem/demo/example_arm.hpp"
#include "lightspeed/subsystem/flag_registry.hpp"
#include "lightspeed/subsystem/scheduler.hpp"
#include "lightspeed/telemetry/dashboard.hpp"
#include "lightspeed/telemetry/sd_logger.hpp"
#include "lightspeed/telemetry/serial_link.hpp"
#include "lightspeed/telemetry/telemetry_bus.hpp"
#include "lightspeed/vision/vision_constants.hpp"
#include "lightspeed/vision/vision_pose_corrector.hpp"
#include "pros/misc.hpp"

namespace {

// -- Shared, program-lifetime hardware/control objects --
//
// initialize() constructs these once; opcontrol() and autonomous() both
// reference the SAME instances. If opcontrol() built its own second
// DrivetrainVelocityController on the same ports, two independent PIDF loops
// would fight over the same physical motors.
std::optional<lightspeed::hal::MotorGroup> gLeftDrive;
std::optional<lightspeed::hal::MotorGroup> gRightDrive;
std::optional<lightspeed::hal::MotorGroup> gIntakeFront;
std::optional<lightspeed::hal::MotorGroup> gIntakeRear;
std::optional<lightspeed::control::DrivetrainVelocityController> gDrivetrain;

std::optional<lightspeed::hal::Imu> gPrimaryImu;
std::optional<lightspeed::hal::Imu> gSecondaryImu;
std::optional<lightspeed::odom::IMESource> gLeftIme;
std::optional<lightspeed::odom::IMESource> gRightIme;
std::optional<lightspeed::odom::IMUSource> gImuSource;
std::optional<lightspeed::odom::OdometryFusion> gOdometry;

std::optional<lightspeed::motion::TurnToHeading> gTurnToHeading;
std::optional<lightspeed::motion::DriveStraightDistance> gDriveStraightDistance;
std::optional<lightspeed::motion::DriveToPoint> gDriveToPoint;
std::optional<lightspeed::motion::MoveToPose> gMoveToPose;
std::optional<lightspeed::motion::PurePursuitController> gPurePursuit;

std::optional<lightspeed::hal::MotorGroup> gExampleArmMotors;
std::optional<lightspeed::subsystem::demo::ExampleArm> gExampleArm;

std::optional<lightspeed::driver::InputProfile> gInputProfile;
std::optional<lightspeed::driver::AccelLimitResolver> gAccelLimitResolver;
std::optional<lightspeed::control::SlewRateLimiter> gLeftAccelSlew;
std::optional<lightspeed::control::SlewRateLimiter> gRightAccelSlew;

// ButtonMacroRunner takes no construction parameters, so it's a plain global
// rather than an optional. gDemoArmMacro captures a reference to
// *gExampleArm, so it must be constructed after it.
lightspeed::driver::ButtonMacroRunner gButtonMacroRunner;
std::optional<lightspeed::driver::ButtonMacro> gDemoArmMacro;

lightspeed::auton::RoutineRegistry gRoutineRegistry;
std::optional<lightspeed::auton::SelectorGui> gSelectorGui;

// TelemetryBus is a singleton, so no shared bus object is constructed here.
std::optional<lightspeed::telemetry::SdLogger> gSdLogger;
std::optional<lightspeed::telemetry::Dashboard> gDashboard;
// Started only inside diagnostic mode, not during competition operation.
std::optional<lightspeed::telemetry::SerialLink> gSerialLink;

// Constructed unconditionally like every other placeholder-port device here,
// but its correction is only ever CONSUMED by diagnostic mode's bench test,
// never applied to gOdometry during normal operation -- applying a
// placeholder-tag-map-derived correction to real match odometry would
// silently corrupt it.
std::optional<lightspeed::hal::AiVisionSensor> gAiVisionSensor;
std::optional<lightspeed::vision::VisionPoseCorrector> gVisionCorrector;

constexpr std::uint32_t kLoopPeriodMs = 10;  // ~100Hz driver-control loop
constexpr double kDtSeconds = kLoopPeriodMs / 1000.0;
constexpr std::uint32_t kStatusIntervalMs = 1000;  // periodic heartbeat, ~1Hz
constexpr std::int32_t kIntakeVoltage = 12000;

double normalizeStick(std::int32_t rawAnalog) {
	return static_cast<double>(rawAnalog) / 127.0;
}

}  // namespace

// Builds every shared object above (IMU calibration blocks ~2-3s here),
// registers the demo routines, and starts the GUI selector so a driver can
// pick a start location and routine any time before the match.
void initialize() {
	using namespace lightspeed;

	gLeftDrive.emplace(hal::config::kLeftDriveGroup);
	gRightDrive.emplace(hal::config::kRightDriveGroup);
	gIntakeFront.emplace(hal::config::kIntakeFrontGroup);
	gIntakeRear.emplace(hal::config::kIntakeRearGroup);
	gDrivetrain.emplace(*gLeftDrive, *gRightDrive, control::kDrivetrainVelocityConfig);

	gPrimaryImu.emplace(hal::config::kPrimaryImu.port);
	gSecondaryImu.emplace(hal::config::kSecondaryImu.port);
	gPrimaryImu->calibrate(true);
	gSecondaryImu->calibrate(true);

	gLeftIme.emplace(*gLeftDrive, odom::kDriveImeConfig);
	gRightIme.emplace(*gRightDrive, odom::kDriveImeConfig);
	gImuSource.emplace(*gPrimaryImu, &*gSecondaryImu);

	// No tracking-wheel pods -- odometry is IME + dual IMU only, so the
	// kinematics fallback runs every cycle.
	gOdometry.emplace(odom::kOdometryTopology.kinematics, *gLeftIme, *gRightIme, *gImuSource,
	                   std::vector<odom::TrackingWheelSource*>{});

	gTurnToHeading.emplace(*gDrivetrain, *gOdometry, motion::kTurnToHeadingConfig);
	gDriveStraightDistance.emplace(*gDrivetrain, *gOdometry, motion::kDriveStraightDistanceConfig);
	gDriveToPoint.emplace(*gTurnToHeading, *gDriveStraightDistance, *gOdometry);
	gMoveToPose.emplace(*gDrivetrain, *gOdometry, motion::kMoveToPoseConfig);
	gPurePursuit.emplace(*gDrivetrain, *gOdometry, motion::kPurePursuitConfig);

	// Must be constructed before AccelLimitResolver so its
	// "exampleArm.isExtended" flag is already registered when that
	// constructor's sanity check runs.
	gExampleArmMotors.emplace(hal::config::kExampleArmGroup);
	gExampleArm.emplace(*gExampleArmMotors);
	subsystem::Scheduler::instance().start();

	gInputProfile.emplace(driver::kInputProfileConfig);
	gAccelLimitResolver.emplace(driver::kDriveAccelLimitConfig);
	gLeftAccelSlew.emplace(driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond);
	gRightAccelSlew.emplace(driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond);

	// Captures *gExampleArm by reference, so must come after it.
	gDemoArmMacro.emplace(driver::demo::makeDemoArmCycleMacro(*gExampleArm));

	gRoutineRegistry.registerRoutine(auton::demo::makeDemoStraightAndTurnRoutine());
	gRoutineRegistry.registerRoutine(auton::demo::makeDemoPursuitPathRoutine());

	gSelectorGui.emplace(*gOdometry, gRoutineRegistry);

	// Both consumed only by diagnostic mode below -- see the globals above.
	gAiVisionSensor.emplace(hal::config::kAiVisionSensor);
	gVisionCorrector.emplace(*gAiVisionSensor, vision::kAiVisionCalibration, vision::kAiVisionMountOffset,
	                          vision::kTagWorldMap, vision::kVisionGatingConfig);
	gSerialLink.emplace(telemetry::TelemetryBus::instance());

	// No-op unless Y was held at boot, in which case it never returns and
	// nothing below starts this boot.
	diagnostics::runDiagnosticModeIfRequested(*gVisionCorrector, *gOdometry, *gSerialLink);

	gSelectorGui->start();

	// Both gate themselves on competition state internally, so starting them
	// here is safe.
	gSdLogger.emplace(telemetry::TelemetryBus::instance());
	gSdLogger->start();
	gDashboard.emplace(*gOdometry, *gExampleArm);
	gDashboard->start();
}

void disabled() {}

void competition_initialize() {}

// Releases the screen from the GUI selector (only one task may draw at a
// time) and runs whichever routine getRoutineForAutonomous() resolves to.
void autonomous() {
	using namespace lightspeed;

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
	    .exampleArm = *gExampleArm,
	};
	routine->run(ctx);

	const odom::Pose finalPose = gOdometry->getPose();
	std::printf("[autonomous] routine complete: final pose=(%.2f, %.2f, %.1fdeg)\n", finalPose.xInches, finalPose.yInches,
	            finalPose.headingDegrees);
}

// Drives the full pipeline: input -> profiling -> drive-mode -> accel-limited
// slew -> velocity controller. R1/R2 move the demo subsystem between presets
// so its `exampleArm.isExtended` flag toggles live and the accel-limit table
// visibly reacts -- watch the console for the accel-limit-change lines.
void opcontrol() {
	using namespace lightspeed;

	// Idempotent -- autonomous() already stops the selector in the normal
	// flow. This covers bench testing where autonomous() never runs, so the
	// selector and dashboard tasks never both hold pros::screen.
	gSelectorGui->stop();

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	double previousAccelLimit = driver::kDriveAccelLimitConfig.defaultMaxRpmPerSecond;
	std::uint32_t previousStatusTime = pros::millis();

	while (true) {
		// -- Input profiling --
		const driver::JoystickInput rawInput{
		    .leftY = -normalizeStick(master.get_analog(ANALOG_LEFT_Y)),
		    .leftX = normalizeStick(master.get_analog(ANALOG_LEFT_X)),
		    .rightY = normalizeStick(master.get_analog(ANALOG_RIGHT_Y)),
		    .rightX = -normalizeStick(master.get_analog(ANALOG_RIGHT_X)),
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
		const double leftSlewedRpm = gLeftAccelSlew->calculate(leftTargetRpm, kDtSeconds);
		const double rightSlewedRpm = gRightAccelSlew->calculate(rightTargetRpm, kDtSeconds);

		// -- The same target-velocity path autonomous uses --
		gDrivetrain->setTargetVelocity(leftSlewedRpm, rightSlewedRpm);

		// Intake controls are direct and hold-to-run so releasing the button
		// immediately stops both motors. L1 takes priority for opposing motion.
		if (master.get_digital(DIGITAL_L1)) {
			gIntakeFront->writeVoltage(kIntakeVoltage);
			gIntakeRear->writeVoltage(-kIntakeVoltage);
		} else if (master.get_digital(DIGITAL_B)) {
			gIntakeFront->writeVoltage(kIntakeVoltage);
			gIntakeRear->writeVoltage(kIntakeVoltage);
		} else if (master.get_digital(DIGITAL_DOWN)) {
			gIntakeFront->writeVoltage(-kIntakeVoltage);
			gIntakeRear->writeVoltage(-kIntakeVoltage);
		} else {
			gIntakeFront->writeVoltage(0);
			gIntakeRear->writeVoltage(0);
		}

		// Suppressed while the L1 macro is running, so a direct press
		// doesn't fight the macro's own moveToPreset() calls.
		if (!gButtonMacroRunner.isRunning()) {
			if (master.get_digital_new_press(DIGITAL_R1)) {
				gExampleArm->moveToPreset("HIGH");
			}
			if (master.get_digital_new_press(DIGITAL_R2)) {
				gExampleArm->moveToPreset("LOW");
			}
		}

		// L1 runs the demo arm-cycle macro without blocking this loop.
		// TODO: L1 is double-bound (intakes above, macro here) -- rebind
		// once a real mechanism replaces the demo arm.
		if (master.get_digital_new_press(DIGITAL_L1)) {
			gButtonMacroRunner.trigger(*gDemoArmMacro);
		}
		gButtonMacroRunner.update();

		if (accelLimit != previousAccelLimit) {
			std::printf("[opcontrol] accel limit changed: %.0f -> %.0f RPM/s (exampleArm.isExtended=%s)\n",
			            previousAccelLimit, accelLimit,
			            subsystem::FlagRegistry::instance().getFlag("exampleArm.isExtended") ? "true" : "false");
			previousAccelLimit = accelLimit;
		}

		const std::uint32_t now = pros::millis();
		if (now - previousStatusTime >= kStatusIntervalMs) {
			std::printf("[opcontrol] target=(%.0f, %.0f)rpm slewed=(%.0f, %.0f)rpm accelLimit=%.0fRPM/s arm=%s\n",
			            leftTargetRpm, rightTargetRpm, leftSlewedRpm, rightSlewedRpm, accelLimit,
			            subsystem::demo::toString(gExampleArm->getState()));
			previousStatusTime = now;
		}

		pros::delay(kLoopPeriodMs);
	}
}
