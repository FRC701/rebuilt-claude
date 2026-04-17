package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Feeds balls from the hopper into the shooter.
 *
 * <p>The roller is gated on the shooter being at target RPM — starting the roller before the
 * shooter is up to speed would cause balls to be launched at unpredictable velocities. The roller
 * runs continuously once the shooter is ready and stops when the command ends (button released).
 *
 * <p>Both Roller and Shooter are required so this command cannot run concurrently with commands
 * that also use either subsystem.
 */
public class RollerFeed extends Command {

    private final Roller m_roller;
    private final Shooter m_shooter;

    public RollerFeed(Roller roller, Shooter shooter) {
        m_roller = roller;
        m_shooter = shooter;
        // Roller is required; Shooter is required to prevent feeding while
        // a SysId or other shooter command is running.
        addRequirements(roller, shooter);
    }

    @Override
    public void execute() {
        // Gate roller on shooter readiness every loop — if the shooter drops
        // below target RPM mid-feed (e.g. from ball load), the roller pauses
        // until speed recovers.
        if (m_shooter.isReadyToFire()) {
            m_roller.runForward();
        } else {
            m_roller.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_roller.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
