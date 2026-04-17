package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.roller.Roller;

/**
 * Runs the roller in reverse to unjam balls from the hopper.
 *
 * <p>Does not require the Shooter subsystem — unjamming can happen independently of shooter state.
 * Runs until interrupted (button released).
 */
public class RollerReverse extends Command {

    private final Roller m_roller;

    public RollerReverse(Roller roller) {
        m_roller = roller;
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        m_roller.runReverse();
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
