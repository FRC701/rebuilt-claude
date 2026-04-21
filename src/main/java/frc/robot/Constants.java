package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.RGBWColor;

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
