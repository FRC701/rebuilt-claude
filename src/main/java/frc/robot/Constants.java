package frc.robot;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

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

        // --- CAN IDs (61–70 range reserved for roller) ---
        public static final int kCANdleID = 61;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kStickDeadband = 0.1;
    }

    /**
     * Field geometry constants for the 2026 Rebuilt game.
     *
     * <p>Hub center coordinates are used by AutoAim for heading calculation and by RobotContainer
     * for distance-based shooter RPM. Centralizing them here prevents duplication and ensures both
     * use the same values.
     */
    public static final class FieldConstants {
        // Hub center coordinates in meters, WPILib blue-alliance-origin
        // field coordinate system (origin = blue alliance wall, left corner).
        // TODO: Look up exact values from the 2026 field dimension drawings.
        public static final Translation2d kBlueHubCenter =
                new Translation2d(Units.inchesToMeters(297.0), Units.inchesToMeters(161.6));
        public static final Translation2d kRedHubCenter =
                new Translation2d(Units.inchesToMeters(354.2), Units.inchesToMeters(161.6));

        /**
         * Returns the current alliance's hub center from FieldConstants. Used by both the right
         * trigger binding and AutoAim so hub position is never duplicated in RobotContainer.
         */
        public static Translation2d getHubCenter() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
                return FieldConstants.kRedHubCenter;
            }
            return FieldConstants.kBlueHubCenter;
        }
    }
}
