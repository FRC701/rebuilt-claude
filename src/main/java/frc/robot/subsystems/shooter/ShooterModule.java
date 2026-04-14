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
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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

    // ── SysId ─────────────────────────────────────────────────────────────────
    // Mutable holders for SysId measurements — reused each loop to avoid GC.
    // Angular units are used since shooter wheels are rotational mechanisms.
    private final MutVoltage m_sysIdAppliedVoltage = Volts.mutable(0);
    private final MutAngle m_sysIdPosition = Rotations.mutable(0);
    private final MutAngularVelocity m_sysIdVelocity = RotationsPerSecond.mutable(0);

    private SysIdRoutine m_sysIdRoutine;

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

    /**
     * Initializes the SysId routine for this module. Must be called after construction, passing the
     * parent subsystem so SysId can properly manage command requirements.
     *
     * @param subsystem The parent Shooter subsystem
     */
    public void initSysId(SubsystemBase subsystem) {
        m_sysIdRoutine =
                new SysIdRoutine(
                        new SysIdRoutine.Config(),
                        new SysIdRoutine.Mechanism(
                                voltage -> m_motor.setVoltage(voltage.in(Volts)),
                                log ->
                                        log.motor(m_name)
                                                .voltage(
                                                        m_sysIdAppliedVoltage.mut_replace(
                                                                m_motor.getMotorVoltage()
                                                                        .getValueAsDouble(),
                                                                Volts))
                                                .angularPosition(
                                                        m_sysIdPosition.mut_replace(
                                                                m_motor.getPosition()
                                                                        .getValueAsDouble(),
                                                                Rotations))
                                                .angularVelocity(
                                                        m_sysIdVelocity.mut_replace(
                                                                m_motor.getVelocity()
                                                                        .getValueAsDouble(),
                                                                RotationsPerSecond)),
                                subsystem));
    }

    /**
     * Returns a SysId quasistatic command for this module. Run slowly in both directions to
     * characterize kS and kV.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a SysId dynamic command for this module. Run with a fast voltage ramp in both
     * directions to characterize kA.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }
}
