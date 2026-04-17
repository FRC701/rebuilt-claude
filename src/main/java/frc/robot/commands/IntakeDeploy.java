package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Deploys the intake arm and runs rollers once the arm reaches the deployed position.
 *
 * <p>Once deployed, the arm oscillates between kDeployedPosition and kDeployedPosition +
 * kBounceAmplitudeRotations on a fixed timer to prevent game pieces from jamming while driving.
 * Timer-based switching is used so the bounce cadence is predictable regardless of PID settle time.
 *
 * <p>Roller start is still gated on isDeployed() so it does not spin against the frame during the
 * initial deploy move.
 */
public class IntakeDeploy extends Command {

    private final Intake m_intake;

    // Timer tracks elapsed time since the arm first reached the deployed position.
    // Restarted when bouncing begins so the first half-cycle starts cleanly.
    private final Timer m_bounceTimer = new Timer();

    // Tracks whether bouncing has started so the timer is only reset once.
    private boolean m_bouncingStarted = false;

    public IntakeDeploy(Intake intake) {
        m_intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        m_bouncingStarted = false;
        m_bounceTimer.stop();
        m_bounceTimer.reset();

        // Begin moving toward the deployed position immediately.
        m_intake.deploy();
    }

    @Override
    public void execute() {
        if (m_intake.isDeployed()) {
            // Start the bounce timer the first time the arm arrives at deployed.
            // Avoids a partial first half-cycle if the arm took time to deploy.
            if (!m_bouncingStarted) {
                m_bouncingStarted = true;
                m_bounceTimer.restart();
            }

            // Update the deploy target each loop based on the timer half-cycle.
            m_intake.deployBounce(m_bounceTimer.get());

            // Roller runs continuously once deployed — bouncing does not affect it.
            m_intake.runRollerForward();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_bounceTimer.stop();
        m_intake.retract();
        m_intake.stopRoller();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
