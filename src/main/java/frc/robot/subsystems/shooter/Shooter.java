package frc.robot.subsystems.shooter;

/**
 * Shooter subsystem manages two independent ShooterModule instances.
 *
 * <p>Design decisions: - Follows the same pattern as the Swerve/SwerveModule architecture.
 * ShooterModule is a plain Java class that encapsulates one motor. This subsystem coordinates both
 * modules and owns all scheduling logic. - setRPMFromDistance() delegates to each module's own
 * lookup table, allowing left and right to have completely independent RPM profiles for spin or
 * curve shots. - isReadyToFire() is centralized here so commands only need to query one place
 * regardless of how many modules are active. - Independent enable/disable is delegated to each
 * ShooterModule so this class stays clean and free of per-motor state tracking.
 */
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    // ── Modules ───────────────────────────────────────────────────────────────
    private final ShooterModule m_left;
    private final ShooterModule m_right;

    public Shooter() {
        m_left = new ShooterModule(ShooterConstants.kLeftConfig);
        m_right = new ShooterModule(ShooterConstants.kRightConfig);

        // Initialize SysId routines — must be called after modules are constructed,
        // passing this subsystem as the requirement owner.
        m_left.initSysId(this);
        m_right.initSysId(this);
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Sets the target RPM for each shooter independently.
     *
     * @param leftRPM Target RPM for the left shooter
     * @param rightRPM Target RPM for the right shooter
     */
    public void setRPM(double leftRPM, double rightRPM) {
        m_left.setRPM(leftRPM);
        m_right.setRPM(rightRPM);
    }

    /**
     * Sets both shooters to the same RPM.
     *
     * @param rpm Target RPM for both shooters
     */
    public void setRPM(double rpm) {
        setRPM(rpm, rpm);
    }

    /**
     * Sets each shooter to the RPM interpolated from the given distance using each module's
     * independent lookup table.
     *
     * @param distanceMeters Distance to the target in meters
     */
    public void setRPMFromDistance(double distanceMeters) {
        m_left.setRPMFromDistance(distanceMeters);
        m_right.setRPMFromDistance(distanceMeters);
    }

    /** Stops both shooters. */
    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    /**
     * Returns true if all enabled shooters are within the velocity tolerance of their target RPM.
     * Use this to gate feeding a game piece.
     */
    public boolean isReadyToFire() {
        return m_left.isAtTargetRPM(ShooterConstants.kVelocityToleranceRPM)
                && m_right.isAtTargetRPM(ShooterConstants.kVelocityToleranceRPM);
    }

    // ── Enable / Disable ──────────────────────────────────────────────────────

    /** Enables the left shooter module. */
    public void enableLeft() {
        m_left.enable();
    }

    /** Disables the left shooter module and stops it immediately. */
    public void disableLeft() {
        m_left.disable();
    }

    /** Enables the right shooter module. */
    public void enableRight() {
        m_right.enable();
    }

    /** Disables the right shooter module and stops it immediately. */
    public void disableRight() {
        m_right.disable();
    }

    /** Returns whether the left shooter module is enabled. */
    public boolean isLeftEnabled() {
        return m_left.isEnabled();
    }

    /** Returns whether the right shooter module is enabled. */
    public boolean isRightEnabled() {
        return m_right.isEnabled();
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    /** Returns the current RPM of the left shooter. */
    public double getLeftRPM() {
        return m_left.getRPM();
    }

    /** Returns the current RPM of the right shooter. */
    public double getRightRPM() {
        return m_right.getRPM();
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter/Left RPM", m_left.getRPM());
        SmartDashboard.putNumber("Shooter/Right RPM", m_right.getRPM());
        SmartDashboard.putNumber("Shooter/Left Target RPM", m_left.getTargetRPM());
        SmartDashboard.putNumber("Shooter/Right Target RPM", m_right.getTargetRPM());
        SmartDashboard.putBoolean("Shooter/Ready To Fire", isReadyToFire());
        SmartDashboard.putBoolean("Shooter/Left Enabled", m_left.isEnabled());
        SmartDashboard.putBoolean("Shooter/Right Enabled", m_right.isEnabled());
    }

    // ── SysId ─────────────────────────────────────────────────────────────────

    /**
     * Returns a SysId quasistatic command for the left shooter module.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdLeftQuasistatic(SysIdRoutine.Direction direction) {
        return m_left.sysIdQuasistatic(direction);
    }

    /**
     * Returns a SysId dynamic command for the left shooter module.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdLeftDynamic(SysIdRoutine.Direction direction) {
        return m_left.sysIdDynamic(direction);
    }

    /**
     * Returns a SysId quasistatic command for the right shooter module.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdRightQuasistatic(SysIdRoutine.Direction direction) {
        return m_right.sysIdQuasistatic(direction);
    }

    /**
     * Returns a SysId dynamic command for the right shooter module.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdRightDynamic(SysIdRoutine.Direction direction) {
        return m_right.sysIdDynamic(direction);
    }
}
