package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterModule;

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

    public static final class ShooterConstants {
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Coast;
        public static final double kSupplyCurrentLimit = 60.0; // amps
        public static final double kStatorCurrentLimit = 120.0; // amps

        public static final InvertedValue kLeftShooterInvert =
                InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue kRightShooterInvert = InvertedValue.Clockwise_Positive;

        // ── Velocity PID (slot 0) ─────────────────────────────────────────────
        // TODO: Tune these after SysId characterization
        public static final double kLeftKP = 0.1;
        public static final double kLeftKI = 0.0;
        public static final double kLeftKD = 0.0;
        public static final double kLeftKS = 0.0; // volts
        public static final double kLeftKV = 0.12; // volts per rot/s
        public static final double kLeftKA = 0.0; // volts per rot/s²

        public static final double kRightKP = 0.1;
        public static final double kRightKI = 0.0;
        public static final double kRightKD = 0.0;
        public static final double kRightKS = 0.0; // volts
        public static final double kRightKV = 0.12; // volts per rot/s
        public static final double kRightKA = 0.0; // volts per rot/s²

        // ── Velocity tolerance ────────────────────────────────────────────────
        // How close to target RPM before we consider the shooter "ready to fire"
        public static final double kVelocityToleranceRPM = 50.0;

        // ── Distance → RPM lookup tables ──────────────────────────────────────
        // TODO: Replace these placeholder values with empirically tested data.
        // Key   = distance to target in meters
        // Value = target RPM for that distance
        // Add as many points as needed — InterpolatingDoubleTreeMap will
        // smoothly interpolate between them at runtime.
        public static final double[][] kLeftRPMMap = {
            {1.0, 2000.0},
            {2.0, 3000.0},
            {3.0, 4000.0},
            {4.0, 5000.0},
            {5.0, 6000.0}
        };

        public static final double[][] kRightRPMMap = {
            {1.0, 2000.0},
            {2.0, 3000.0},
            {3.0, 4000.0},
            {4.0, 5000.0},
            {5.0, 6000.0}
        };

        // ── Module configs ────────────────────────────────────────────────────
        public static final ShooterModule.Config kLeftConfig =
                new ShooterModule.Config(
                        "Left",
                        CANDevices.kLeftShooterID,
                        CANDevices.kRioBus,
                        kLeftShooterInvert,
                        kNeutralMode,
                        kLeftKP,
                        kLeftKI,
                        kLeftKD,
                        kLeftKS,
                        kLeftKV,
                        kLeftKA,
                        kSupplyCurrentLimit,
                        kStatorCurrentLimit,
                        kLeftRPMMap);

        public static final ShooterModule.Config kRightConfig =
                new ShooterModule.Config(
                        "Right",
                        CANDevices.kRightShooterID,
                        CANDevices.kRioBus,
                        kRightShooterInvert,
                        kNeutralMode,
                        kRightKP,
                        kRightKI,
                        kRightKD,
                        kRightKS,
                        kRightKV,
                        kRightKA,
                        kSupplyCurrentLimit,
                        kStatorCurrentLimit,
                        kRightRPMMap);
    }

    /**
     * Intake hardware constants.
     *
     * <p>Deploy motor uses position control so the arm holds at a known angle. Roller motor uses
     * open-loop duty cycle — no need for closed-loop on a simple intake roller. Both motors are on
     * the RIO bus because the CANivore is already carrying the swerve hardware and keeping
     * high-frequency devices together there is cleaner.
     */
    public static final class IntakeConstants {

        // --- Motor config ---
        // Brake on deploy keeps the arm from back-driving under gravity/impact.
        // Brake on roller ensures it stops quickly when command ends.
        public static final NeutralModeValue kDeployNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kRollerNeutralMode = NeutralModeValue.Brake;

        // Inversion — verify on first power-up; positive output should deploy outward.
        public static final boolean kDeployInverted = false;
        public static final boolean kRollerInverted = false;

        // --- Current limits ---
        // Supply limit protects wiring; stator limit prevents mechanism damage.
        public static final double kDeploySupplyCurrentLimit = 40.0; // amps
        public static final double kDeployStatorCurrentLimit = 60.0; // amps
        public static final double kRollerSupplyCurrentLimit = 30.0; // amps
        public static final double kRollerStatorCurrentLimit = 40.0; // amps

        // --- Deploy positions (rotations of the motor shaft) ---
        // TODO: Measure empirically — jog motor and read sensor position in Tuner X.
        public static final double kDeployedPosition = 10.0; // rotations — placeholder
        public static final double kRetractedPosition = 0.0; // rotations — home/zero

        // --- Deploy PID gains (Slot 0) ---
        // Higher kP pushes through linkage slop quickly on the way out.
        // TODO: Tune with manual sweep. Start ~0.5, increase until arm moves
        // crisply without oscillating.
        //
        // --- Gravity feedforward ---
        // kG is a constant voltage (volts) added to the output to counteract gravity.
        // Tune by finding the minimum voltage that holds the arm stationary at the
        // hardest point against gravity (usually mid-travel). Start at 0.0 and
        // increase in small steps (~0.1V) until the arm holds without drifting.
        // TODO: Tune empirically — kG will likely be higher for retract (lifting)
        // than deploy (lowering) if the arm geometry changes mechanical advantage
        // across the range of motion.
        public static final double kDeployKP = 0.5;
        public static final double kDeployKI = 0.0; // I term rarely needed for position
        public static final double kDeployKD = 0.0;
        public static final double kDeployKG = 0.0; // volts — placeholder

        // --- Retract PID gains (Slot 1) ---
        // Softer kP on retract — mechanism is being pulled back and slop is
        // taken up in the opposite direction, so a hard snap is less useful
        // and risks slamming the arm against the retracted hard stop.
        // TODO: Start lower than kDeployKP (~0.3) and tune separately.
        public static final double kRetractKP = 0.3;
        public static final double kRetractKI = 0.0;
        public static final double kRetractKD = 0.0;
        public static final double kRetractKG = 0.0; // volts — placeholder

        // Position tolerance — arm is considered "at target" within this many rotations.
        public static final double kDeployToleranceRotations = 0.5;

        // --- Bounce constants ---
        // Bouncing oscillates the deploy arm between two positions while intaking
        // to prevent game pieces from jamming. Timer-based switching is used so
        // the arm moves predictably regardless of PID settle time.
        //
        // kBounceAmplitudeRotations defines how far above kDeployedPosition the arm
        // travels on the upstroke — keep small enough that the arm stays clearly
        // deployed but large enough to dislodge jams.
        // TODO: Tune empirically — start small (~1.0 rotation) and increase if
        // bouncing is not effective.
        public static final double kBounceAmplitudeRotations = 1.0; // rotations above deployed
        public static final double kBounceHalfPeriodSeconds = 0.3; // seconds per half-cycle

        // --- Roller speeds ---
        // Duty cycle (-1.0 to 1.0). Positive = intaking, negative = ejecting.
        public static final double kRollerForwardSpeed = 0.8;
        public static final double kRollerReverseSpeed = -0.8;
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

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final double kStickDeadband = 0.1;
    }
}
