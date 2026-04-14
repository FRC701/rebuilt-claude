package frc.robot.commands;

/**
 * Waits until the shooter is ready to fire, then finishes immediately.
 *
 * <p>Design decisions: - This command does not require the shooter subsystem so it can run in
 * parallel with ShooterSetRPM or ShooterSetRPMFromDistance inside a ParallelRaceGroup or
 * SequentialCommandGroup without conflicting. - Intended to be used as a gate in auto routines: new
 * ShooterSetRPM(m_shooter, 4000) .alongWith(new WaitUntilShooterReady(m_shooter)) .andThen(new
 * AgitatorForward(m_agitator)); - A timeout should be added in RobotContainer when used in auto to
 * prevent the robot from waiting forever if a shooter is disabled or malfunctioning:
 * command.withTimeout(3.0)
 */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;

public class WaitUntilShooterReady extends Command {

    private final Shooter m_shooter;

    /**
     * Creates a new WaitUntilShooterReady command.
     *
     * @param shooter The shooter subsystem
     */
    public WaitUntilShooterReady(Shooter shooter) {
        m_shooter = shooter;
        // No addRequirements() — this command only reads state, does not control
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return m_shooter.isReadyToFire();
    }
}
