package frc.robot;

/**
 * RobotContainer wires together all subsystems, commands, and operator inputs.
 *
 * <p>Design decisions: - A single {@link edu.wpi.first.wpilibj2.command.CommandXboxController} is
 * used for the driver. Add a second controller here for an operator. - {@link TeleopSwerve} is set
 * as the default command for the swerve subsystem so it runs automatically during teleop whenever
 * no other command requires the drivetrain. - PathPlanner's {@code AutoBuilder} is configured here
 * during construction. All auto routines are registered via {@code NamedCommands} and selected via
 * a {@code SendableChooser} on SmartDashboard. - The gyro zero button (Back/Select) resets
 * field-relative heading. Useful when the robot's field orientation drifts during a match. - All
 * button bindings use {@code CommandXboxController} trigger/button methods with WPILib's {@code
 * Trigger} API for clean, declarative bindings.
 */
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoAim;
import frc.robot.commands.IntakeDeploy;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.RollerFeed;
import frc.robot.commands.RollerReverse;
import frc.robot.commands.ShooterSetRPM;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Agitator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;

public class RobotContainer {

    // ── Subsystems ────────────────────────────────────────────────────────────
    private final Agitator m_agitator = new Agitator();
    private final Shooter m_shooter = new Shooter();
    private final Swerve m_swerve = new Swerve();
    private final Intake m_intake = new Intake();
    private final Roller m_roller = new Roller();
    private final Vision m_vision = new Vision();

    // ── Controllers ───────────────────────────────────────────────────────────
    private final CommandXboxController m_driverController =
            new CommandXboxController(OIConstants.kDriverControllerPort);

    // ── Auto chooser ──────────────────────────────────────────────────────────
    private final SendableChooser<Command> m_autoChooser;

    public RobotContainer() {
        m_swerve.setVision(m_vision);

        configurePathPlanner();
        configureDefaultCommands();
        configureButtonBindings();

        // Build auto chooser from all autos defined in the deploy/pathplanner/autos
        // directory. The argument is the default auto to select on boot.
        m_autoChooser = AutoBuilder.buildAutoChooser("None");
        SmartDashboard.putData("Auto Chooser", m_autoChooser);
    }

    // ── PathPlanner ───────────────────────────────────────────────────────────

    private void configurePathPlanner() {
        try {
            // Load robot physical config from PathPlanner's GUI export.
            // TODO: Configure your robot in the PathPlanner GUI and export
            // the config, or adjust RobotConfig parameters manually.
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    m_swerve::getPose,
                    m_swerve::resetPose,
                    m_swerve::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> m_swerve.drive(speeds, false),
                    new PPHolonomicDriveController(
                            // TODO: Tune translation PID gains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // TODO: Tune rotation PID gains
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    // Flip paths for red alliance automatically
                    () -> {
                        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
                        return alliance.isPresent()
                                && alliance.get()
                                        == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
                    },
                    m_swerve);

        } catch (Exception e) {
            System.err.println(
                    "[RobotContainer] Failed to configure PathPlanner: " + e.getMessage());
        }

        // ── Register named commands ───────────────────────────────────────────
        // TODO: Register commands that PathPlanner autos can invoke by name.
        // Example:
        // NamedCommands.registerCommand("IntakeDown", new IntakeDownCommand(m_intake));
        NamedCommands.registerCommand("Placeholder", Commands.none());
    }

    // ── Default Commands ──────────────────────────────────────────────────────

    private void configureDefaultCommands() {
        m_swerve.setDefaultCommand(new TeleopSwerve(m_swerve, m_driverController));
    }

    // ── Button Bindings ───────────────────────────────────────────────────────

    private void configureButtonBindings() {
        // Back/Select button → zero gyro heading
        m_driverController.back().onTrue(Commands.runOnce(m_swerve::zeroGyro, m_swerve));

        // Button bindings — add to configureButtonBindings()
        // whileTrue retracts and stops roller automatically when button is released,
        // triggering IntakeDeploy.end().
        m_driverController.rightBumper().toggleOnTrue(new IntakeDeploy(m_intake));
        m_driverController.leftBumper().toggleOnTrue(new IntakeReverse(m_intake));

        m_driverController
                .leftTrigger()
                .whileTrue(new AutoAim(m_swerve, m_shooter, m_roller, m_driverController));

        // RollerFeed gates on shooter RPM internally, so binding it alongside
        // ShooterSetRPM on the same trigger makes sense.
        m_driverController.povDown().whileTrue(new RollerReverse(m_roller));

        // Spin up the shooter and feed balls simultaneously.
        // ShooterSetRPM owns Shooter; RollerFeed owns Roller.
        // RollerFeed internally gates on isAtTargetRPM() so balls only
        // feed once the shooter is ready — no explicit sequencing needed.
        m_driverController
                .rightTrigger()
                .whileTrue(
                        new ShooterSetRPM(m_shooter, Shooter.ShooterConstants.kDefaultRPM)
                                .alongWith(new RollerFeed(m_roller, m_shooter)));

        // ── SysId bindings (comment out during normal use) ────────────────────────
        // Hold LB + press A/B/X/Y to run SysId routines.
        // LB must be held as a safety interlock so routines don't run accidentally.
        m_driverController
                .leftBumper()
                .and(m_driverController.a())
                .whileTrue(m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.b())
                .whileTrue(m_swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_driverController
                .leftBumper()
                .and(m_driverController.x())
                .whileTrue(m_swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.y())
                .whileTrue(m_swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // ── Shooter SysId bindings (comment out during normal use) ────────────────
        // Left shooter: Hold LB + RB + press A/B/X/Y
        // Right shooter: Hold LB + RB + Start + press A/B/X/Y
        // Double interlock (LB + RB) prevents accidental activation during matches.
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.a())
                .whileTrue(m_shooter.sysIdLeftQuasistatic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.b())
                .whileTrue(m_shooter.sysIdLeftQuasistatic(SysIdRoutine.Direction.kReverse));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.x())
                .whileTrue(m_shooter.sysIdLeftDynamic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.y())
                .whileTrue(m_shooter.sysIdLeftDynamic(SysIdRoutine.Direction.kReverse));

        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.start())
                .and(m_driverController.a())
                .whileTrue(m_shooter.sysIdRightQuasistatic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.start())
                .and(m_driverController.b())
                .whileTrue(m_shooter.sysIdRightQuasistatic(SysIdRoutine.Direction.kReverse));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.start())
                .and(m_driverController.x())
                .whileTrue(m_shooter.sysIdRightDynamic(SysIdRoutine.Direction.kForward));
        m_driverController
                .leftBumper()
                .and(m_driverController.rightBumper())
                .and(m_driverController.start())
                .and(m_driverController.y())
                .whileTrue(m_shooter.sysIdRightDynamic(SysIdRoutine.Direction.kReverse));

        // TODO: Add more button bindings here as subsystems are added.
        // Examples:
        // m_driverController.a().whileTrue(new IntakeCommand(m_intake));
        // m_driverController.rightBumper().onTrue(new ShootCommand(m_shooter));
    }

    // ── Auto ──────────────────────────────────────────────────────────────────

    /**
     * Called by {@link Robot} to get the selected autonomous command.
     *
     * @return The command to run during autonomous
     */
    public Command getAutonomousCommand() {
        return m_autoChooser.getSelected();
    }
}
