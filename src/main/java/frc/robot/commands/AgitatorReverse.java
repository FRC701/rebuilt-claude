package frc.robot.commands;

/**
 * Runs both agitators in reverse at the default speed while the command is scheduled, and stops
 * them when it ends.
 *
 * <p>Design decisions: - Runs in reverse for outtaking game pieces or clearing jams. - Mirrors
 * AgitatorForward in structure for consistency — both commands are simple wrappers that delegate to
 * the subsystem. - isFinished() returns false so the command runs until interrupted, making it
 * suitable for whileTrue() button bindings. - stop() is called in end() regardless of whether the
 * command was interrupted or finished normally.
 */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;

public class AgitatorReverse extends Command {

    private final Agitator m_agitator;

    /**
     * Creates a new AgitatorReverse command.
     *
     * @param agitator The agitator subsystem
     */
    public AgitatorReverse(Agitator agitator) {
        m_agitator = agitator;
        addRequirements(agitator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_agitator.reverse();
    }

    @Override
    public void end(boolean interrupted) {
        m_agitator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
