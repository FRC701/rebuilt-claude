package frc.robot.subsystems.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Low-level wrapper around a single TalonFX for the roller subsystem.
 *
 * <p>Plain Java class (not SubsystemBase) following the project's ShooterModule and IntakeModule
 * patterns — hardware details are encapsulated here so {@link Roller} stays at a higher
 * coordination level.
 *
 * <p>Open-loop duty cycle only — no closed-loop needed for a feed roller whose only job is to move
 * balls at a consistent rate.
 */
public class RollerModule {

    /**
     * Immutable configuration record for a RollerModule.
     *
     * <p>Record pattern keeps call sites readable and all constants co-located in Constants.java.
     */
    public record Config(
            int canId,
            CANBus bus,
            boolean inverted,
            NeutralModeValue neutralMode,
            double supplyCurrentLimit,
            double statorCurrentLimit) {}

    private final TalonFX m_motor;

    // Pre-allocated control request — reused every loop to avoid GC pressure.
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0).withEnableFOC(false);

    public RollerModule(Config config) {
        m_motor = new TalonFX(config.canId(), config.bus());

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = config.neutralMode();
        cfg.MotorOutput.Inverted =
                config.inverted()
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;

        // Supply limit protects wiring; stator limit protects motor/gearbox.
        cfg.CurrentLimits.SupplyCurrentLimitEnable = true;
        cfg.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit();
        cfg.CurrentLimits.StatorCurrentLimitEnable = true;
        cfg.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit();

        m_motor.getConfigurator().apply(cfg);
    }

    /** Sets motor output as open-loop duty cycle. */
    public void setDutyCycle(double dutyCycle) {
        m_motor.setControl(m_dutyCycleRequest.withOutput(dutyCycle));
    }

    /** Stops the motor, applying the configured neutral mode. */
    public void stop() {
        m_motor.stopMotor();
    }
}
