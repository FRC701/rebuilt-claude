package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class CANDevices {
        public static final CANBus kRioBus = new CANBus("rio");
        public static final CANBus kCanIvoreBus = new CANBus("canivore");

        // Drive Motors (TalonFX)
        public static final int kFrontLeftDriveID = 1;
        public static final int kFrontRightDriveID = 2;
        public static final int kBackLeftDriveID = 3;
        public static final int kBackRightDriveID = 4;

        // Steer Motors (TalonFX)
        public static final int kFrontLeftSteerID = 5;
        public static final int kFrontRightSteerID = 6;
        public static final int kBackLeftSteerID = 7;
        public static final int kBackRightSteerID = 8;

        // CANcoders
        public static final int kFrontLeftCanCoderID = 1;
        public static final int kFrontRightCanCoderID = 2;
        public static final int kBackLeftCanCoderID = 3;
        public static final int kBackRightCanCoderID = 4;

        // Pigeon 2
        public static final int kPigeonID = 1;

        // Agitator Motors (TalonFX) - RoboRIO CAN bus
        public static final int kLeftAgitatorID = 21;
        public static final int kRightAgitatorID = 22;

        // Shooter Motors (TalonFX) - RoboRIO CAN bus
        public static final int kLeftShooterID = 31;
        public static final int kRightShooterID = 32;

        // --- CAN IDs (41–50 range reserved for intake) ---
        public static final int kIntakeDeployId = 41;
        public static final int kIntakeRollerId = 42;

        // --- CAN IDs (51–60 range reserved for roller) ---
        public static final int kLeftRollerMotorId = 51;
        public static final int kRightRollerMotorId = 52;
    }

    /**
     * Roller subsystem constants.
     *
     * <p>The roller moves balls from the hopper into the shooter using two open-loop duty cycle
     * motors, one per side. Reverse is supported for unjamming. CAN IDs 51-60 are reserved for the
     * roller subsystem.
     */
    public static final class RollerConstants {
        // --- Motor config ---
        // Brake neutral mode stops balls immediately when command ends,
        // preventing balls from drifting into the shooter unintentionally.
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Inversion — verify on first power-up. Both motors should move
        // balls toward the shooter when running forward.
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true; // typically mirrored

        // --- Current limits ---
        public static final double kSupplyCurrentLimit = 30.0; // amps
        public static final double kStatorCurrentLimit = 40.0; // amps

        // --- Roller speeds ---
        // Duty cycle (-1.0 to 1.0).
        // TODO: Tune empirically — start conservative and increase until
        // balls feed reliably without jamming.
        public static final double kForwardSpeed = 0.8;
        public static final double kReverseSpeed = -0.8;
    }

    /**
     * Vision constants for PhotonVision pose estimation and target tracking.
     *
     * <p>A single camera handles both AprilTag pose estimation (fed into SwerveDrivePoseEstimator)
     * and target tracking (used for auto-aim). The robot-to-camera transform must be measured
     * carefully — even small errors in the transform will cause pose estimation drift.
     */
    public static final class VisionConstants {
        // Camera name must match the name configured in the PhotonVision UI.
        public static final String kCameraName = "photonvision"; // TODO: update to match UI

        // Robot-to-camera transform — describes where the camera is mounted
        // relative to the robot center. Measure from robot center (floor level)
        // to camera lens.
        // TODO: Measure all three values precisely on the physical robot.
        // Translation: forward(+X), left(+Y), up(+Z) in meters
        // Rotation: roll, pitch, yaw in radians
        public static final Transform3d kRobotToCamera =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(
                                        12.0), // forward from robot center — placeholder
                                Units.inchesToMeters(0.0), // left/right offset — placeholder
                                Units.inchesToMeters(24.0)), // height from floor — placeholder
                        new Rotation3d(
                                0.0, // roll — typically 0
                                Math.toRadians(
                                        -30.0), // pitch — tilt down toward targets, placeholder
                                0.0)); // yaw — 0 if camera faces forward

        // AprilTag field layout — used by PoseEstimationStrategy to compute
        // robot pose from tag detections. Load from the WPILib field layout
        // for the current game.
        public static final AprilTagFieldLayout kFieldLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Maximum ambiguity ratio — discard pose estimates above this threshold.
        // Lower = stricter filtering. 0.2 is a reasonable starting point.
        public static final double kMaxAmbiguity = 0.2;

        // Maximum distance from current estimated pose to accept a vision
        // measurement. Rejects wildly wrong estimates caused by tag misreads.
        public static final double kMaxPoseJumpMeters = 1.0;
    }

    /**
     * LED constants for the CANdle strip.
     *
     * <p>A single CANdle drives the full LED strip. Priority order for state display (highest to
     * lowest): shooter ready, intake deployed, default (red). This means shooter ready always wins
     * if multiple conditions are true simultaneously.
     */
    public static final class LEDConstants {
        public static final int kCANdleID = 60; // TODO: confirm CAN ID
        public static final int kNumLEDs = 64; // TODO: confirm LED count

        // --- Colors (R, G, B) ---
        // Shooter ready — green signals the driver it is safe to feed.
        public static final RGBWColor kShooterReadyColor = new RGBWColor(0, 255, 0); // green

        // Intake deployed — orange signals the driver the intake is out.
        public static final RGBWColor kIntakeDeployedColor = new RGBWColor(255, 165, 0); // orange

        // Default — red when no special condition is active.
        public static final RGBWColor kDefaultColor = new RGBWColor(255, 0, 0); // red
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kStickDeadband = 0.1;
    }
}
