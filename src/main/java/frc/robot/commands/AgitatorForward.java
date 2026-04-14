package frc.robot.commands;

/**
 * Runs both agitators forward at the default speed while the command is scheduled, and stops them
 * when it ends.
 *
 * <p>Design decisions: - Extends Command directly rather than using Commands.run() inline in
 * RobotContainer so the intent is explicit and the command is reusable across multiple button
 * bindings and auto routines. - Speed is driven by AgitatorConstants.kDefaultSpeed so tuning is
 * centralized in Constants.java. - isFinished() returns false so the command runs until
 * interrupted, making it suitable for whileTrue() button bindings. - stop() is called in end()
 * regardless of whether the command was interrupted or finished normally.
 */
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Agitator;

public class AgitatorForward extends Command {

    private final Agitator m_agitator;

    /**
     * Creates a new AgitatorForward command.
     *
     * @param agitator The agitator subsystem
     */
    public AgitatorForward(Agitator agitator) {
        m_agitator = agitator;
        addRequirements(agitator);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_agitator.run();
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
