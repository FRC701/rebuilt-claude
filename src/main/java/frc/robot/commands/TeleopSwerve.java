package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.Swerve;

/*
 * A few design decisions worth noting:
 *
 * * Slew rate limiters smooth out rapid stick movements to reduce wheel slip
 *   and improve feel — the 0.25s ramp time is a reasonable starting point but
 *   your driver may want it faster or slower
 * * Input squaring (Math.copySign(x * x, x)) gives finer control at low speeds
 *   while still reaching full speed — this is a very common FRC driving technique
 * * initialize() resets the slew limiters so there's no unwanted ramp-up when
 *   the command restarts after an auto routine
 * * The axis negations (-getLeftY(), -getLeftX(), -getRightX()) are standard
 *   for Xbox controllers where forward/left are negative
 *
 */

public class TeleopSwerve extends Command {

    // ── Dependencies ──────────────────────────────────────────────────────────
    private final Swerve m_swerve;
    private final CommandXboxController m_controller;

    // ── Slew rate limiters ────────────────────────────────────────────────────
    // Limits how quickly the driver can accelerate, reducing wheel slip and
    // improving control feel. Units are (units/second).
    // TODO: Tune these values to match your driver's preference.
    private final SlewRateLimiter m_xLimiter =
            new SlewRateLimiter(SwerveConstants.kTeleopMaxSpeedMPS / 0.25); // 0→full in 0.25s
    private final SlewRateLimiter m_yLimiter =
            new SlewRateLimiter(SwerveConstants.kTeleopMaxSpeedMPS / 0.25);
    private final SlewRateLimiter m_rotLimiter =
            new SlewRateLimiter(SwerveConstants.kTeleopMaxAngularRPS / 0.25);

    /**
     * Creates a new TeleopSwerve command.
     *
     * @param swerve The swerve drivetrain subsystem
     * @param controller The driver's Xbox controller
     */
    public TeleopSwerve(Swerve swerve, CommandXboxController controller) {
        m_swerve = swerve;
        m_controller = controller;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Reset slew limiters so there's no ramp-up from a stale state
        m_xLimiter.reset(0);
        m_yLimiter.reset(0);
        m_rotLimiter.reset(0);
    }

    @Override
    public void execute() {
        // ── Read raw joystick inputs ──────────────────────────────────────────
        // Left stick Y → forward/back (negate because forward is negative on Xbox)
        // Left stick X → strafe left/right
        // Right stick X → rotation
        double rawX = -m_controller.getLeftY();
        double rawY = -m_controller.getLeftX();
        double rawRot = -m_controller.getRightX();

        // ── Apply deadband ────────────────────────────────────────────────────
        double x = MathUtil.applyDeadband(rawX, OIConstants.kStickDeadband);
        double y = MathUtil.applyDeadband(rawY, OIConstants.kStickDeadband);
        double rot = MathUtil.applyDeadband(rawRot, OIConstants.kStickDeadband);

        // ── Square inputs for finer low-speed control ─────────────────────────
        // Preserves sign while squaring magnitude, giving more precision
        // near center while still allowing full speed at full stick.
        x = Math.copySign(x * x, x);
        y = Math.copySign(y * y, y);
        rot = Math.copySign(rot * rot, rot);

        // ── Scale to physical units ───────────────────────────────────────────
        double xMPS = m_xLimiter.calculate(x * SwerveConstants.kTeleopMaxSpeedMPS);
        double yMPS = m_yLimiter.calculate(y * SwerveConstants.kTeleopMaxSpeedMPS);
        double rotRPS = m_rotLimiter.calculate(rot * SwerveConstants.kTeleopMaxAngularRPS);

        // ── Send to drivetrain ────────────────────────────────────────────────
        m_swerve.drive(new ChassisSpeeds(xMPS, yMPS, rotRPS));
    }

    @Override
    public void end(boolean interrupted) {
        m_swerve.stop();
    }

    @Override
    public boolean isFinished() {
        // TeleopSwerve runs until interrupted (e.g. by an auto command)
        return false;
    }
}
