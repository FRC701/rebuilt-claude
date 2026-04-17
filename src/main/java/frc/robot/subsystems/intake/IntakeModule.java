package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeConstants;

/**
 * Low-level wrapper around a single TalonFX for the intake.
 *
 * <p>Plain Java class (not SubsystemBase) following the project's Swerve/SwerveModule and
 * Shooter/ShooterModule patterns — hardware details are encapsulated here so {@link Intake} stays
 * at a higher coordination level.
 *
 * <p>A {@link Config} record replaces a long constructor parameter list, keeping call sites
 * readable and constants co-located in {@code Constants.java}.
 */
public class IntakeModule {

    /**
     * Immutable configuration record for an IntakeModule.
     *
     * <p>Using a record here (same pattern as ShooterModule.Config) means adding a new tuning
     * parameter only requires changing this record and Constants — not every call site.
     */
    public record Config(
            int canId,
            CANBus bus,
            boolean inverted,
            NeutralModeValue neutralMode,
            double supplyCurrentLimit,
            double statorCurrentLimit) {}

    private final TalonFX m_motor;

    // Pre-allocated control requests — reused every loop to avoid GC pressure.
    // Phoenix 6 best practice: create once, mutate with withX() calls.
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(false);
    private final PositionVoltage m_positionRequest =
            new PositionVoltage(0.0).withEnableFOC(false).withSlot(0);

    public IntakeModule(Config config) {
        m_motor = new TalonFX(config.canId(), config.bus());

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        // Neutral mode — brake keeps deploy arm from back-driving.
        cfg.MotorOutput.NeutralMode = config.neutralMode();
        cfg.MotorOutput.Inverted =
                config.inverted()
                        ? com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive
                        : com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;

        // Current limits — supply protects wiring, stator protects motor/gearbox.
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit();

        // PID slot 0 — used by the deploy motor for position control.
        // Roller module will never call setPosition(), so these gains are harmless.
        cfg.Slot0.kP = IntakeConstants.kDeployKP;
        cfg.Slot0.kI = IntakeConstants.kDeployKI;
        cfg.Slot0.kD = IntakeConstants.kDeployKD;

        // Slot 1 — retract direction, softer gains to avoid slamming the hard stop.
        cfg.Slot1.kP = IntakeConstants.kRetractKP;
        cfg.Slot1.kI = IntakeConstants.kRetractKI;
        cfg.Slot1.kD = IntakeConstants.kRetractKD;

        m_motor.getConfigurator().apply(cfg);
    }

    /** Sets motor output as open-loop duty cycle. Used for the roller. */
    public void setDutyCycle(double dutyCycle) {
        m_motor.setControl(m_dutyCycleRequest.withOutput(dutyCycle));
    }

    /**
     * Moves motor to a target position using the specified PID slot.
     *
     * <p>Slot 0 is used for deploy (higher gains to push through slop), slot 1 for retract (softer
     * gains to avoid slamming the hard stop). Keeping slot selection at this level lets Intake.java
     * express intent (deploy vs retract) without knowing PID implementation details.
     */
    public void setPosition(double rotations, int slot) {
        m_motor.setControl(m_positionRequest.withPosition(rotations).withSlot(slot));
    }

    /** Seeds the internal encoder with the current physical position as zero. */
    public void seedEncoder() {
        m_motor.setPosition(0.0);
    }

    /** Returns current motor position in rotations (relative encoder). */
    public double getPosition() {
        return m_motor.getPosition().getValueAsDouble();
    }

    /** Stops the motor, applying the configured neutral mode. */
    public void stop() {
        m_motor.stopMotor();
    }
}
