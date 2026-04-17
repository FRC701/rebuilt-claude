package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Runs the intake rollers in reverse to eject a game piece.
 *
 * <p>Does not move the deploy arm — ejecting is only useful when the arm is already deployed. If
 * called while retracted the roller will spin harmlessly. Runs until interrupted (button released).
 */
public class IntakeReverse extends Command {

    private final Intake m_intake;

    public IntakeReverse(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_intake.runRollerReverse();
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
