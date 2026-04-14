package frc.robot.commands;

/**
 * Spins both shooters up to the RPM interpolated from the given distance and holds that speed until
 * interrupted.
 *
 * <p>Design decisions: - Accepts distance as a DoubleSupplier rather than a fixed value so the
 * command always uses the latest distance from vision or odometry while running, automatically
 * adjusting RPM if the robot moves. - isFinished() returns false — runs until interrupted. - stop()
 * is called in end() so shooters spin down when released.
 *
 * <p>DoubleSupplier is a Java functional interface — it is a way to pass a method that returns a
 * double rather than a fixed double value. The lambda is evaluated every loop while the command
 * runs, rather than once when the command is created.
 *
 * <p>Usage examples (in RobotContainer):
 *
 * <p>// Distance from a vision helper method: new ShooterSetRPMFromDistance(m_shooter, () ->
 * m_vision.getDistanceToSpeaker());
 *
 * <p>// Distance calculated from odometry pose: new ShooterSetRPMFromDistance(m_shooter, () ->
 * m_swerve.getPose().getTranslation() .getDistance(FieldConstants.kSpeakerPosition));
 *
 * <p>// Fixed distance for testing: new ShooterSetRPMFromDistance(m_shooter, () -> 3.0);
 *
 * <p>// Distance from SmartDashboard entry (useful for tuning): new
 * ShooterSetRPMFromDistance(m_shooter, () -> SmartDashboard.getNumber("Shooter/Test Distance",
 * 2.0));
 *
 * <p>Note: In a match, Example 2 is the most common pattern — the robot pose from odometry or
 * vision gives a Translation2d, and getDistance() computes the straight-line distance to a fixed
 * field position such as the speaker.
 */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ShooterSetRPMFromDistance extends Command {

    private final Shooter m_shooter;
    private final DoubleSupplier m_distanceSupplier;

    /**
     * Creates a new ShooterSetRPMFromDistance command.
     *
     * @param shooter The shooter subsystem
     * @param distanceSupplier Supplier returning the current distance to the target in meters (e.g.
     *     from vision or odometry)
     */
    public ShooterSetRPMFromDistance(Shooter shooter, DoubleSupplier distanceSupplier) {
        m_shooter = shooter;
        m_distanceSupplier = distanceSupplier;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_shooter.setRPMFromDistance(m_distanceSupplier.getAsDouble());
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
