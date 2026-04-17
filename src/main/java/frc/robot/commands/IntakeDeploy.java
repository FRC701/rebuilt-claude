package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Deploys the intake arm and runs rollers once the arm reaches the deployed position.
 *
 * <p>Sequencing deploy-then-roll here (rather than starting both simultaneously) prevents the
 * roller from spinning against the frame during the deploy movement, reducing wear and current
 * draw. The command runs indefinitely until interrupted, at which point {@code end()} retracts the
 * arm and stops the roller.
 *
 * <p>This is a continuous command (extends Command, not InstantCommand) because it needs to hold
 * position and keep rollers running until the button is released.
 */
public class IntakeDeploy extends Command {

    private final Intake m_intake;

    public IntakeDeploy(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Start moving the arm toward the deployed position immediately.
        m_intake.deploy();
    }

    @Override
    public void execute() {
        // Only run the roller once the arm is actually out — avoids spinning
        // against the robot frame during the deploy motion.
        if (m_intake.isDeployed()) {
            m_intake.runRollerForward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Retract and stop regardless of whether we were interrupted or finished.
        m_intake.retract();
        m_intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        // Never self-finish — runs until the button is released (interrupted).
        return false;
    }
}
