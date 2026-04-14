package frc.robot.commands;

/**
 * Spins both shooters up to a fixed RPM and holds that speed until interrupted.
 *
 * <p>Design decisions: - Takes an explicit RPM parameter rather than reading from constants so the
 * same command class can be reused for different speed presets (e.g. close shot, far shot) simply
 * by passing a different value. - isFinished() returns false — the command runs until interrupted,
 * making it suitable for whileTrue() bindings or as the first step in a SequentialCommandGroup that
 * waits for isReadyToFire(). - stop() is called in end() so the shooter spins down when released.
 */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSetRPM extends Command {

    private final Shooter m_shooter;
    private final double m_leftRPM;
    private final double m_rightRPM;

    /**
     * Creates a new ShooterSetRPM command.
     *
     * @param shooter The shooter subsystem
     * @param leftRPM Target RPM for the left shooter
     * @param rightRPM Target RPM for the right shooter
     */
    public ShooterSetRPM(Shooter shooter, double leftRPM, double rightRPM) {
        m_shooter = shooter;
        m_leftRPM = leftRPM;
        m_rightRPM = rightRPM;
        addRequirements(shooter);
    }

    /**
     * Convenience constructor for setting both shooters to the same RPM.
     *
     * @param shooter The shooter subsystem
     * @param rpm Target RPM for both shooters
     */
    public ShooterSetRPM(Shooter shooter, double rpm) {
        this(shooter, rpm, rpm);
    }

    @Override
    public void initialize() {
        m_shooter.setRPM(m_leftRPM, m_rightRPM);
    }

    @Override
    public void execute() {
        // RPM is set in initialize() — nothing to do each loop
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
