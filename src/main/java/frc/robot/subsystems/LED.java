package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

/**
 * LED subsystem — drives a CANdle strip to communicate robot state to drivers.
 *
 * <p>Uses the Phoenix 6 CANdle API — LEDs are controlled via {@code setControl()} with a {@link
 * SolidColor} control request, consistent with the rest of the project's Phoenix 6 usage. The
 * Phoenix 5 {@code setLEDs()} and {@code configAllSettings()} API is not used.
 *
 * <p>State priority (highest to lowest):
 *
 * <ol>
 *   <li>Shooter ready — green (safe to feed)
 *   <li>Intake deployed — orange (intake is out)
 *   <li>Default — red (no special condition active)
 * </ol>
 *
 * <p>State is evaluated every loop in periodic() so LEDs always reflect current robot condition
 * without any explicit command needed. Last-state tracking avoids redundant CAN writes every loop.
 */
public class LED extends SubsystemBase {

    private final CANdle m_candle;
    private final Shooter m_shooter;
    private final Intake m_intake;

    // Pre-allocated control request — reused every loop to avoid GC pressure,
    // consistent with TalonFX control request pattern used throughout this project.
    // Range is fixed at construction (full strip, 0 to kNumLEDs) — only color
    // changes between calls so withColor() mutation is safe to reuse.
    private final SolidColor m_solidColorRequest = new SolidColor(0, LEDConstants.kNumLEDs);

    // Track last state to avoid redundant CANdle writes — CAN bus writes
    // should be minimized to reduce bus load.
    private LEDState m_lastState = null;

    /** All possible LED states in priority order. */
    private enum LEDState {
        SHOOTER_READY,
        INTAKE_DEPLOYED,
        DEFAULT
    }

    public LED(Shooter shooter, Intake intake) {
        m_shooter = shooter;
        m_intake = intake;

        m_candle = new CANdle(LEDConstants.kCANdleID, CANDevices.kRioBus);

        // Configure strip type and disable onboard status LED when actively
        // controlling — uses Phoenix 6 configurator pattern consistent with
        // TalonFX/CANcoder configuration elsewhere in this project.
        CANdleConfiguration config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB; // TODO: verify strip type
        config.LED.BrightnessScalar = 1.0;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        m_candle.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        LEDState desiredState = getDesiredState();

        // Only write to CANdle when state changes — avoids saturating CAN bus
        // with identical commands every 20ms loop.
        if (desiredState == m_lastState) {
            return;
        }

        m_lastState = desiredState;

        switch (desiredState) {
            case SHOOTER_READY:
                setColor(LEDConstants.kShooterReadyColor);
                break;
            case INTAKE_DEPLOYED:
                setColor(LEDConstants.kIntakeDeployedColor);
                break;
            case DEFAULT:
            default:
                setColor(LEDConstants.kDefaultColor);
                break;
        }
    }

    // ── Private Helpers ───────────────────────────────────────────────────────

    /**
     * Evaluates current subsystem states and returns the highest-priority LED state that should be
     * displayed.
     */
    private LEDState getDesiredState() {
        if (m_shooter.isReadyToFire()) {
            return LEDState.SHOOTER_READY;
        }
        if (m_intake.isDeployed()) {
            return LEDState.INTAKE_DEPLOYED;
        }
        return LEDState.DEFAULT;
    }

    /**
     * Sets the entire LED strip to a solid color using the Phoenix 6 SolidColor control request.
     *
     * <p>SolidColor takes start index and LED count in the constructor. A new instance is created
     * per call since the range is fixed at construction time — unlike TalonFX control requests
     * which are mutable and reused.
     *
     * @param r Red (0–255)
     * @param g Green (0–255)
     * @param b Blue (0–255)
     */
    private void setColor(RGBWColor color) {
        m_candle.setControl(m_solidColorRequest.withColor(color));
    }
}
