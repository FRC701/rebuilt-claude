package frc.robot;

/**
 * Robot is the top-level class for the WPILib command-based framework.
 *
 * <p>Design decisions: - This class is intentionally minimal. All subsystems, commands, and
 * bindings live in {@link RobotContainer}, keeping Robot.java as a thin lifecycle wrapper only. -
 * {@code autonomousInit()} resets the robot pose to the starting pose of the selected auto path by
 * scheduling the auto command immediately. PathPlanner's {@code AutoBuilder} handles the pose reset
 * internally on the first path. - {@code teleopInit()} cancels any running auto command to prevent
 * it from interfering with driver control. - {@code testInit()} cancels all running commands for
 * safety when entering test mode.
 */
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;

public class Robot extends TimedRobot {

    private RobotContainer m_robotContainer;
    private Command m_autonomousCommand;

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        // Runs the command scheduler. This must be called every loop cycle
        // for commands, triggers, and default commands to work.
        CommandScheduler.getInstance().run();
    }

    // ── Disabled ──────────────────────────────────────────────────────────────

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    // ── Autonomous ────────────────────────────────────────────────────────────

    @Override
    public void autonomousInit() {
        FieldConstants.clearHubCenterCache();
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    // ── Teleop ────────────────────────────────────────────────────────────────

    @Override
    public void teleopInit() {
        FieldConstants.clearHubCenterCache();
        // Cancel auto command when teleop starts so the driver has full control
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    // ── Test ──────────────────────────────────────────────────────────────────

    @Override
    public void testInit() {
        // Cancel all running commands for safety when entering test mode
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    // ── Simulation ────────────────────────────────────────────────────────────

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
