package frc.robot.subsystems;

/**
 * Agitator subsystem controls a left and right agitator motor.
 *
 * <p>Design decisions: - Both motors are TalonFX on the RoboRIO CAN bus (not CANivore), so the
 * default empty string "" is used as the bus name. - The right motor is inverted relative to the
 * left so both agitators move in the same direction when given the same speed command. - Control is
 * percent output via DutyCycleOut, which is the appropriate Phoenix 6 request for open-loop speed
 * control. - A single speed value drives both motors together since they mirror each other.
 * Individual overrides are provided for special cases such as clearing jams. - Brake mode is used
 * so the agitators stop quickly when commanded off, preventing game pieces from drifting through.
 */
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AgitatorConstants;
import frc.robot.Constants.CANDevices;

public class Agitator extends SubsystemBase {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX m_leftMotor;
    private final TalonFX m_rightMotor;

    // ── Control Requests ──────────────────────────────────────────────────────
    private final DutyCycleOut m_dutyCycleRequest = new DutyCycleOut(0.0);

    public Agitator() {
        // ── Left Motor ────────────────────────────────────────────────────────
        m_leftMotor = new TalonFX(CANDevices.kLeftAgitatorID, CANDevices.kRioBus);

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = AgitatorConstants.kLeftAgitatorInvert;
        leftConfig.MotorOutput.NeutralMode = AgitatorConstants.kNeutralMode;
        leftConfig.CurrentLimits.SupplyCurrentLimit = AgitatorConstants.kSupplyCurrentLimit;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.StatorCurrentLimit = AgitatorConstants.kStatorCurrentLimit;
        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_leftMotor.getConfigurator().apply(leftConfig);

        // ── Right Motor ───────────────────────────────────────────────────────
        m_rightMotor = new TalonFX(CANDevices.kRightAgitatorID, CANDevices.kRioBus);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = AgitatorConstants.kRightAgitatorInvert;
        rightConfig.MotorOutput.NeutralMode = AgitatorConstants.kNeutralMode;
        rightConfig.CurrentLimits.SupplyCurrentLimit = AgitatorConstants.kSupplyCurrentLimit;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.StatorCurrentLimit = AgitatorConstants.kStatorCurrentLimit;
        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        m_rightMotor.getConfigurator().apply(rightConfig);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Runs both agitators at the given speed.
     *
     * @param speed Percent output (-1.0 to 1.0). Positive moves game pieces in the intended
     *     direction.
     */
    public void run(double speed) {
        m_leftMotor.setControl(m_dutyCycleRequest.withOutput(speed));
        m_rightMotor.setControl(m_dutyCycleRequest.withOutput(speed));
    }

    /**
     * Runs both agitators at the default speed defined in {@link
     * frc.robot.Constants.AgitatorConstants#kDefaultSpeed}.
     */
    public void run() {
        run(AgitatorConstants.kDefaultSpeed);
    }

    /**
     * Runs the agitators in reverse at the given speed. Useful for clearing jams.
     *
     * @param speed Percent output (0.0 to 1.0) — sign is applied internally
     */
    public void reverse(double speed) {
        run(-Math.abs(speed));
    }

    /** Reverses both agitators at the default speed. */
    public void reverse() {
        reverse(AgitatorConstants.kDefaultSpeed);
    }

    /**
     * Runs the left and right agitators at independent speeds. Useful for steering game pieces or
     * clearing asymmetric jams.
     *
     * @param leftSpeed Percent output for the left motor (-1.0 to 1.0)
     * @param rightSpeed Percent output for the right motor (-1.0 to 1.0)
     */
    public void runIndependent(double leftSpeed, double rightSpeed) {
        m_leftMotor.setControl(m_dutyCycleRequest.withOutput(leftSpeed));
        m_rightMotor.setControl(m_dutyCycleRequest.withOutput(rightSpeed));
    }

    /** Stops both agitators. */
    public void stop() {
        run(0.0);
    }

    /** Returns true if either agitator motor is drawing above its supply limit. */
    public boolean isStalled() {
        return m_leftMotor.getSupplyCurrent().getValueAsDouble()
                        >= AgitatorConstants.kSupplyCurrentLimit
                || m_rightMotor.getSupplyCurrent().getValueAsDouble()
                        >= AgitatorConstants.kSupplyCurrentLimit;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Telemetry can be added here later if needed
    }
}
