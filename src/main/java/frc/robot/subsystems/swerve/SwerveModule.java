package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {

    // ── Hardware ─────────────────────────────────────────────────────────────
    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;
    private final CANcoder m_canCoder;

    // ── Control Requests ─────────────────────────────────────────────────────
    // VelocityVoltage: closed-loop velocity control with feedforward for drive
    private final VelocityVoltage m_driveVelocityRequest =
            new VelocityVoltage(0).withSlot(0).withEnableFOC(false);

    // PositionVoltage: closed-loop position control for steer
    private final PositionVoltage m_steerPositionRequest =
            new PositionVoltage(0).withSlot(0).withEnableFOC(false);

    // ── Module name for debugging ─────────────────────────────────────────────
    private final String m_name;

    /**
     * Constructs a SwerveModule.
     *
     * @param name Human-readable name (e.g. "Front Left") for debugging
     * @param driveMotorID CAN ID of the drive TalonFX
     * @param steerMotorID CAN ID of the steer TalonFX
     * @param canCoderID CAN ID of the CANcoder
     * @param canCoderOffset Absolute offset of the CANcoder when wheel points forward
     * @param driveInverted Whether the drive motor is inverted
     */
    public SwerveModule(
            String name,
            int driveMotorID,
            int steerMotorID,
            int canCoderID,
            Rotation2d canCoderOffset,
            InvertedValue driveInverted) {

        m_name = name;

        // ── CANcoder ──────────────────────────────────────────────────────────
        m_canCoder = new CANcoder(canCoderID, CANDevices.kCanIvoreBus);

        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.MagnetOffset = canCoderOffset.getRotations();
        canCoderConfig.MagnetSensor.SensorDirection = SwerveConstants.kCanCoderDirection;
        m_canCoder.getConfigurator().apply(canCoderConfig);

        // ── Steer Motor ───────────────────────────────────────────────────────
        m_steerMotor = new TalonFX(steerMotorID, CANDevices.kCanIvoreBus);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.MotorOutput.Inverted = SwerveConstants.kSteerInvert;
        steerConfig.MotorOutput.NeutralMode = SwerveConstants.kSteerNeutralMode;

        // Use CANcoder as remote sensor for absolute steer position
        steerConfig.Feedback.FeedbackRemoteSensorID = canCoderID;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        steerConfig.Feedback.SensorToMechanismRatio = 1.0;
        steerConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;

        // Steer PID (slot 0)
        steerConfig.Slot0.kP = SwerveConstants.kSteerKP;
        steerConfig.Slot0.kI = SwerveConstants.kSteerKI;
        steerConfig.Slot0.kD = SwerveConstants.kSteerKD;

        // Current limits
        steerConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kSteerSupplyCurrentLimit;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kSteerStatorCurrentLimit;
        steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Wrap steer position so the module never rotates more than 180 degrees
        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        m_steerMotor.getConfigurator().apply(steerConfig);

        // ── Drive Motor ───────────────────────────────────────────────────────
        m_driveMotor = new TalonFX(driveMotorID, CANDevices.kCanIvoreBus);

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = driveInverted;
        driveConfig.MotorOutput.NeutralMode = SwerveConstants.kDriveNeutralMode;

        // Drive PID + feedforward (slot 0)
        driveConfig.Slot0.kP = SwerveConstants.kDriveKP;
        driveConfig.Slot0.kI = SwerveConstants.kDriveKI;
        driveConfig.Slot0.kD = SwerveConstants.kDriveKD;
        driveConfig.Slot0.kS = SwerveConstants.kDriveKS;
        driveConfig.Slot0.kV = SwerveConstants.kDriveKV;
        driveConfig.Slot0.kA = SwerveConstants.kDriveKA;

        // Current limits
        driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kDriveSupplyCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveStatorCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        m_driveMotor.getConfigurator().apply(driveConfig);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Commands the module to the desired state (speed + angle). Applies cosine scaling to reduce
     * wheel scrub when the steer angle error is large.
     *
     * @param desiredState The desired {@link SwerveModuleState}
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize: flip drive direction if it means a smaller steer rotation
        SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, getSteerAngle());

        // Cosine scaling: reduce drive speed proportional to steer angle error
        optimized.speedMetersPerSecond *= optimized.angle.minus(getSteerAngle()).getCos();

        // Convert m/s to rotations/s for TalonFX velocity control
        double driveRotationsPerSecond =
                optimized.speedMetersPerSecond
                        / SwerveConstants.kWheelCircumference
                        * SwerveConstants.kDriveGearRatio;

        m_driveMotor.setControl(m_driveVelocityRequest.withVelocity(driveRotationsPerSecond));

        m_steerMotor.setControl(
                m_steerPositionRequest.withPosition(optimized.angle.getRotations()));
    }

    /** Returns the current state of the module (velocity + angle). Used for telemetry. */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMPS(), getSteerAngle());
    }

    /** Returns the current position of the module (distance + angle). Used for odometry. */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePositionMeters(), getSteerAngle());
    }

    /** Resets the drive encoder position to zero. Call during robot initialization. */
    public void resetDriveEncoder() {
        m_driveMotor.setPosition(0.0);
    }

    /** Returns the module name for debugging. */
    public String getName() {
        return m_name;
    }

    /**
     * Directly sets the drive motor output voltage. Used by SysId routines only — do not call
     * during normal operation.
     *
     * @param volts Voltage to apply (-12 to 12)
     */
    public void setDriveVoltage(double volts) {
        m_driveMotor.setVoltage(volts);
        // Hold steer motor straight forward during SysId
        m_steerMotor.setControl(m_steerPositionRequest.withPosition(0.0));
    }

    /** Returns the drive motor applied voltage. Used by SysId. */
    public double getDriveVoltage() {
        return m_driveMotor.getMotorVoltage().getValueAsDouble();
    }

    /** Returns the drive position in meters. Used by SysId. */
    public double getDrivePositionMeters() {
        return m_driveMotor.getPosition().getValueAsDouble()
                / SwerveConstants.kDriveGearRatio
                * SwerveConstants.kWheelCircumference;
    }

    /** Returns the drive velocity in m/s. Used by SysId. */
    public double getDriveVelocityMPS() {
        return m_driveMotor.getVelocity().getValueAsDouble()
                / SwerveConstants.kDriveGearRatio
                * SwerveConstants.kWheelCircumference;
    }

    // ── Private Helpers ───────────────────────────────────────────────────────

    public Rotation2d getSteerAngle() {
        // RemoteCANcoder reports position directly in rotations
        return Rotation2d.fromRotations(m_steerMotor.getPosition().getValueAsDouble());
    }
}
