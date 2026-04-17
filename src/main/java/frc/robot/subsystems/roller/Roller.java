package frc.robot.subsystems.roller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.RollerConstants;

/**
 * Roller subsystem — feeds balls from the hopper into the shooter.
 *
 * <p>Coordinates two RollerModules (left and right) running open-loop. The roller is intended to
 * run only when the shooter has reached its target RPM — that gating is handled at the command
 * level via {@link frc.robot.commands.RollerFeed}, which uses {@link
 * frc.robot.subsystems.shooter.Shooter#isAtTargetRPM()} as a condition before starting the roller.
 */
public class Roller extends SubsystemBase {

    private final RollerModule m_leftModule;
    private final RollerModule m_rightModule;

    public Roller() {
        m_leftModule =
                new RollerModule(
                        new RollerModule.Config(
                                CANDevices.kLeftRollerMotorId,
                                CANDevices.kRioBus,
                                RollerConstants.kLeftInverted,
                                RollerConstants.kNeutralMode,
                                RollerConstants.kSupplyCurrentLimit,
                                RollerConstants.kStatorCurrentLimit));

        m_rightModule =
                new RollerModule(
                        new RollerModule.Config(
                                CANDevices.kRightRollerMotorId,
                                CANDevices.kRioBus,
                                RollerConstants.kRightInverted,
                                RollerConstants.kNeutralMode,
                                RollerConstants.kSupplyCurrentLimit,
                                RollerConstants.kStatorCurrentLimit));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Roller/Running", m_running);
    }

    // Tracks whether the roller is actively running — used for telemetry.
    private boolean m_running = false;

    /** Runs both rollers forward to feed balls into the shooter. */
    public void runForward() {
        m_running = true;
        m_leftModule.setDutyCycle(RollerConstants.kForwardSpeed);
        m_rightModule.setDutyCycle(RollerConstants.kForwardSpeed);
    }

    /** Runs both rollers in reverse to unjam. */
    public void runReverse() {
        m_running = true;
        m_leftModule.setDutyCycle(RollerConstants.kReverseSpeed);
        m_rightModule.setDutyCycle(RollerConstants.kReverseSpeed);
    }

    /** Stops both roller motors. */
    public void stop() {
        m_running = false;
        m_leftModule.stop();
        m_rightModule.stop();
    }
}
