package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShooterConstants;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Swerve.SwerveConstants;

/**
 * AutoAim rotates the robot to face the alliance hub using field pose, then fires automatically
 * once aligned and the shooter is at target RPM.
 *
 * <p>Design decisions:
 *
 * <ul>
 *   <li>Uses {@code SwerveDrivePoseEstimator} pose rather than camera yaw — works even when no
 *       AprilTag is visible, and is more stable since pose is continuously fused from odometry and
 *       vision.
 *   <li>Target heading is computed each loop from the robot's current field position and the known
 *       hub center coordinates, so it stays accurate as the robot translates while aiming.
 *   <li>A {@link ProfiledPIDController} is used for rotation to limit angular acceleration —
 *       prevents the robot from spinning out when the heading error is large.
 *   <li>Alliance hub is selected from DriverStation each loop so it always reflects the correct
 *       alliance even without FMS.
 *   <li>The driver retains full translational control while auto-aim overrides rotation only.
 *   <li>Firing is gated on BOTH alignment and shooter readiness.
 *   <li>The command never self-finishes — runs until button is released, allowing multiple shots
 *       per button hold.
 * </ul>
 */
public class AutoAim extends Command {

    private final Swerve m_swerve;
    private final Shooter m_shooter;
    private final Roller m_roller;
    private final CommandXboxController m_controller;

    // Inside AutoAim.java
    public static final class Constants {
        public static final double kAutoAimKP = 0.05;
        public static final double kAutoAimKI = 0.0;
        public static final double kAutoAimKD = 0.0;
        public static final double kAutoAimToleranceDegrees = 2.0;
    }

    // Profiled PID controller limits angular acceleration on large heading
    // corrections, preventing wheel scrub and pose estimate degradation
    // from a sudden spin.
    // TODO: Tune maxVelocity and maxAcceleration alongside kP.
    private final ProfiledPIDController m_headingController =
            new ProfiledPIDController(
                    Constants.kAutoAimKP,
                    Constants.kAutoAimKI,
                    Constants.kAutoAimKD,
                    new TrapezoidProfile.Constraints(
                            SwerveConstants.kMaxAngularVelocityRPS, // max rad/s
                            SwerveConstants.kMaxAngularVelocityRPS * 2)); // max rad/s²

    public AutoAim(
            Swerve swerve, Shooter shooter, Roller roller, CommandXboxController controller) {
        m_swerve = swerve;
        m_shooter = shooter;
        m_roller = roller;
        m_controller = controller;

        addRequirements(swerve, shooter, roller);

        // Enable continuous input so the controller always takes the shortest
        // path to the target heading rather than spinning the long way around.
        m_headingController.enableContinuousInput(-Math.PI, Math.PI);
        m_headingController.setTolerance(Math.toRadians(Constants.kAutoAimToleranceDegrees));
    }

    @Override
    public void initialize() {
        // Seed the controller with the current heading so it doesn't
        // produce a large output spike on the first loop.
        m_headingController.reset(m_swerve.getGyroYaw().getRadians());
    }

    @Override
    public void execute() {
        // ── Compute desired heading to hub ────────────────────────────────────
        Translation2d hubCenter = getHubCenter();
        Translation2d robotPosition = m_swerve.getPose().getTranslation();

        // Vector from robot to hub — atan2 gives the field-relative angle.
        double dx = hubCenter.getX() - robotPosition.getX();
        double dy = hubCenter.getY() - robotPosition.getY();
        double targetHeadingRadians = Math.atan2(dy, dx);

        // ── Rotation correction via ProfiledPID ───────────────────────────────
        double rotationCorrection =
                m_headingController.calculate(
                        m_swerve.getGyroYaw().getRadians(), targetHeadingRadians);

        // ── Translation — driver retains full control ─────────────────────────
        double xSpeed = -m_controller.getLeftY() * SwerveConstants.kTeleopMaxSpeedMPS;
        double ySpeed = -m_controller.getLeftX() * SwerveConstants.kTeleopMaxSpeedMPS;

        m_swerve.drive(new ChassisSpeeds(xSpeed, ySpeed, rotationCorrection), true);

        // ── Shooter spin-up ───────────────────────────────────────────────────
        // Spin up every loop so the shooter is ready as soon as alignment
        // is achieved — don't wait until aligned to start spinning.
        m_shooter.setRPM(ShooterConstants.kDefaultRPM);

        // ── Fire — only when aligned AND shooter is ready ─────────────────────
        boolean aligned = m_headingController.atSetpoint();
        boolean shooterReady = m_shooter.isReadyToFire();

        if (aligned && shooterReady) {
            m_roller.runForward();
        } else {
            m_roller.stop();
        }

        // ── Telemetry ─────────────────────────────────────────────────────────
        SmartDashboard.putBoolean("AutoAim/Aligned", aligned);
        SmartDashboard.putBoolean("AutoAim/ShooterReady", shooterReady);
        SmartDashboard.putBoolean("AutoAim/Firing", aligned && shooterReady);
        SmartDashboard.putNumber("AutoAim/TargetHeadingDeg", Math.toDegrees(targetHeadingRadians));
        SmartDashboard.putNumber(
                "AutoAim/HeadingErrorDeg", Math.toDegrees(m_headingController.getPositionError()));
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
        m_shooter.stop();
        m_roller.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // ── Private Helpers ───────────────────────────────────────────────────────

    /**
     * Returns the field position of the current alliance's hub center. Defaults to blue hub if
     * alliance is unknown — safe for testing without FMS connected.
     */
    private Translation2d getHubCenter() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return FieldConstants.kRedHubCenter;
        }
        return FieldConstants.kBlueHubCenter;
    }
}
