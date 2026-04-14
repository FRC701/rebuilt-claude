package frc.robot.subsystems.shooter;

/**
 * ShooterModule represents a single shooter motor with its own PID/FF configuration, enable/disable
 * state, and distance-to-RPM lookup table.
 *
 * <p>Design decisions: - This is a plain Java class, not a SubsystemBase, mirroring the
 * SwerveModule pattern. The Shooter subsystem manages multiple ShooterModule instances and is the
 * only SubsystemBase in the shooter system. - Each module owns its own InterpolatingDoubleTreeMap
 * so left and right shooters can have completely independent distance-to-RPM profiles. -
 * Enable/disable state is encapsulated here so the Shooter subsystem does not need to track it
 * externally. - Coast mode is used so shooter wheels spin down naturally after stopping, avoiding
 * mechanical stress from abrupt braking. - FOC is disabled since we are not using a Phoenix Pro
 * license.
 */
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterModule {

    // ── Hardware ──────────────────────────────────────────────────────────────
    private final TalonFX m_motor;

    // ── Control Requests ──────────────────────────────────────────────────────
    private final VelocityVoltage m_velocityRequest =
            new VelocityVoltage(0.0).withSlot(0).withEnableFOC(false);

    // ── State ─────────────────────────────────────────────────────────────────
    private boolean m_enabled = true;
    private double m_targetRPM = 0.0;
    private final String m_name;

    // ── Distance → RPM lookup table ───────────────────────────────────────────
    private final InterpolatingDoubleTreeMap m_rpmMap = new InterpolatingDoubleTreeMap();

    /**
     * Constructs a ShooterModule.
     *
     * @param name Human-readable name (e.g. "Left", "Right") for telemetry
     * @param motorID CAN ID of the TalonFX motor
     * @param canBus CAN bus the motor is on
     * @param inverted Motor inversion
     * @param neutralMode Motor neutral mode
     * @param kP Velocity PID proportional gain
     * @param kI Velocity PID integral gain
     * @param kD Velocity PID derivative gain
     * @param kS Feedforward static friction (volts)
     * @param kV Feedforward velocity gain (volts per rot/s)
     * @param kA Feedforward acceleration gain (volts per rot/s²)
     * @param supplyLimit Supply current limit (amps)
     * @param statorLimit Stator current limit (amps)
     * @param rpmMap Distance-to-RPM lookup table data (meters → RPM)
     */
    public ShooterModule(
            String name,
            int motorID,
            CANBus canBus,
            InvertedValue inverted,
            NeutralModeValue neutralMode,
            double kP,
            double kI,
            double kD,
            double kS,
            double kV,
            double kA,
            double supplyLimit,
            double statorLimit,
            double[][] rpmMap) {

        m_name = name;

        // ── Populate lookup table ─────────────────────────────────────────────
        for (double[] point : rpmMap) {
            m_rpmMap.put(point[0], point[1]);
        }

        // ── Motor configuration ───────────────────────────────────────────────
        m_motor = new TalonFX(motorID, canBus);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = neutralMode;
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = statorLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        m_motor.getConfigurator().apply(config);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Sets the target RPM for this module. Has no effect if the module is disabled.
     *
     * @param rpm Target RPM
     */
    public void setRPM(double rpm) {
        m_targetRPM = rpm;
        if (m_enabled) {
            m_motor.setControl(m_velocityRequest.withVelocity(rpmToRotationsPerSecond(rpm)));
        }
    }

    /**
     * Sets the target RPM by interpolating from the given distance. Has no effect if the module is
     * disabled.
     *
     * @param distanceMeters Distance to the target in meters
     */
    public void setRPMFromDistance(double distanceMeters) {
        setRPM(m_rpmMap.get(distanceMeters));
    }

    /** Stops this module. Has no effect if the module is disabled. */
    public void stop() {
        m_targetRPM = 0.0;
        if (m_enabled) {
            m_motor.stopMotor();
        }
    }

    /**
     * Returns true if this module is within the velocity tolerance of its target RPM, or if it is
     * disabled.
     *
     * @param toleranceRPM Acceptable RPM error
     */
    public boolean isAtTargetRPM(double toleranceRPM) {
        if (!m_enabled) {
            return true;
        }
        return Math.abs(getRPM() - m_targetRPM) <= toleranceRPM;
    }

    // ── Enable / Disable ──────────────────────────────────────────────────────

    /** Enables this module. */
    public void enable() {
        m_enabled = true;
    }

    /**
     * Disables this module and stops its motor immediately. The module will ignore all subsequent
     * setRPM() calls until re-enabled.
     */
    public void disable() {
        m_enabled = false;
        m_motor.stopMotor();
    }

    /** Returns whether this module is enabled. */
    public boolean isEnabled() {
        return m_enabled;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    /** Returns the current RPM of this module. */
    public double getRPM() {
        return rotationsPerSecondToRPM(m_motor.getVelocity().getValueAsDouble());
    }

    /** Returns the target RPM of this module. */
    public double getTargetRPM() {
        return m_targetRPM;
    }

    /** Returns the name of this module. */
    public String getName() {
        return m_name;
    }

    // ── Private Helpers ───────────────────────────────────────────────────────

    private double rpmToRotationsPerSecond(double rpm) {
        return rpm / 60.0;
    }

    private double rotationsPerSecondToRPM(double rps) {
        return rps * 60.0;
    }
}
