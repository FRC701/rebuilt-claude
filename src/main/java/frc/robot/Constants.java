package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
    }

    public static final class SwerveConstants {

        // ── Wheel & Module Geometry ──────────────────────────────────────────
        // TODO: Measure the actual center-to-center distance between modules
        // on your robot (in meters) and update these values.
        public static final double kTrackWidth = Units.inchesToMeters(22.5); // left-right
        public static final double kWheelBase = Units.inchesToMeters(22.5); // front-back

        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kWheelCircumference = Math.PI * kWheelDiameterMeters;

        // SDS MK4i L3 drive gear ratio
        public static final double kDriveGearRatio = 6.12;

        // SDS MK4i steer gear ratio
        public static final double kSteerGearRatio = 150.0 / 7.0; // ~21.43:1

        // ── Kinematics ───────────────────────────────────────────────────────
        // Module order: Front Left, Front Right, Back Left, Back Right
        public static final SwerveDriveKinematics kKinematics =
                new SwerveDriveKinematics(
                        new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0), // Front Left
                        new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0), // Front Right
                        new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0), // Back Left
                        new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0) // Back Right
                        );

        // ── Speed Limits ─────────────────────────────────────────────────────
        // Kraken X60 free speed: ~6000 RPM
        public static final double kMaxDriveVelocityMPS =
                (6000.0 / 60.0) / kDriveGearRatio * kWheelCircumference; // ~5.49 m/s (~18 ft/s)

        public static final double kMaxAngularVelocityRPS =
                Math.PI * 2; // rad/s (~1 full rotation/sec)

        // Teleop input scaling
        public static final double kTeleopMaxSpeedMPS = kMaxDriveVelocityMPS;
        public static final double kTeleopMaxAngularRPS = kMaxAngularVelocityRPS;

        // ── CANcoder Offsets ─────────────────────────────────────────────────
        // TODO: Use Phoenix Tuner X to find the absolute position of each
        // CANcoder when the wheel is physically pointed straight forward,
        // then enter those values here (in rotations).
        public static final Rotation2d kFrontLeftCanCoderOffset =
                Rotation2d.fromRotations(0.0); // TODO
        public static final Rotation2d kFrontRightCanCoderOffset =
                Rotation2d.fromRotations(0.0); // TODO
        public static final Rotation2d kBackLeftCanCoderOffset =
                Rotation2d.fromRotations(0.0); // TODO
        public static final Rotation2d kBackRightCanCoderOffset =
                Rotation2d.fromRotations(0.0); // TODO

        // ── Motor Inversion ──────────────────────────────────────────────────
        // TODO: Verify these on your physical robot. Typically on MK4i,
        // left-side drive motors are counter-clockwise positive.
        public static final InvertedValue kFrontLeftDriveInvert =
                InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kFrontRightDriveInvert = InvertedValue.Clockwise_Positive;
        public static final InvertedValue kBackLeftDriveInvert =
                InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kBackRightDriveInvert = InvertedValue.Clockwise_Positive;

        // Steer motors are the same across all MK4i modules
        public static final InvertedValue kSteerInvert = InvertedValue.Clockwise_Positive;

        // CANcoder direction (MK4i standard)
        public static final SensorDirectionValue kCanCoderDirection =
                SensorDirectionValue.CounterClockwise_Positive;

        // ── Neutral Modes ────────────────────────────────────────────────────
        public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kSteerNeutralMode = NeutralModeValue.Coast;

        // ── Current Limits (Kraken X60) ──────────────────────────────────────
        public static final double kDriveSupplyCurrentLimit = 60.0; // amps
        public static final double kDriveStatorCurrentLimit = 120.0; // amps
        public static final double kSteerSupplyCurrentLimit = 30.0; // amps
        public static final double kSteerStatorCurrentLimit = 60.0; // amps

        // ── Drive PID & Feedforward (velocity control) ───────────────────────
        // TODO: Tune these values using SysId or empirically on the robot.
        public static final double kDriveKP = 0.1;
        public static final double kDriveKI = 0.0;
        public static final double kDriveKD = 0.0;
        public static final double kDriveKS = 0.0; // volts (static friction)
        public static final double kDriveKV = 0.12; // volts per rot/s
        public static final double kDriveKA = 0.0; // volts per rot/s²

        // ── Steer PID ────────────────────────────────────────────────────────
        // TODO: Tune these values. Start with kP around 50-100 for TalonFX
        // position control.
        public static final double kSteerKP = 100.0;
        public static final double kSteerKI = 0.0;
        public static final double kSteerKD = 0.5;
    }

    public static final class AgitatorConstants {
        public static final InvertedValue kLeftAgitatorInvert =
                InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kRightAgitatorInvert = InvertedValue.Clockwise_Positive;

        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        public static final double kSupplyCurrentLimit = 40.0; // amps
        public static final double kStatorCurrentLimit = 80.0; // amps

        // TODO: Tune this to the desired agitator speed (0.0 to 1.0)
        public static final double kDefaultSpeed = 0.5;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kStickDeadband = 0.1;
    }
}
