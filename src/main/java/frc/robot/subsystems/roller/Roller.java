package frc.robot.subsystems.roller;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

/**
 * Roller subsystem — feeds balls from the hopper into the shooter.
 *
 * <p>Coordinates two RollerModules (left and right) running open-loop. The roller is intended to
 * run only when the shooter has reached its target RPM — that gating is handled at the command
 * level via {@link frc.robot.commands.RollerFeed}, which uses {@link
 * frc.robot.subsystems.shooter.Shooter#isAtTargetRPM()} as a condition before starting the roller.
 */
public class Roller extends SubsystemBase {

    /**
     * Roller subsystem constants.
     *
     * <p>The roller moves balls from the hopper into the shooter using two open-loop duty cycle
     * motors, one per side. Reverse is supported for unjamming. CAN IDs 51-60 are reserved for the
     * roller subsystem.
     */
    public static final class RollerConstants {
        // --- Motor config ---
        // Brake neutral mode stops balls immediately when command ends,
        // preventing balls from drifting into the shooter unintentionally.
        public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;

        // Inversion — verify on first power-up. Both motors should move
        // balls toward the shooter when running forward.
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true; // typically mirrored

        // --- Current limits ---
        public static final double kSupplyCurrentLimit = 30.0; // amps
        public static final double kStatorCurrentLimit = 40.0; // amps

        // --- Roller speeds ---
        // Duty cycle (-1.0 to 1.0).
        // TODO: Tune empirically — start conservative and increase until
        // balls feed reliably without jamming.
        public static final double kForwardSpeed = 0.8;
        public static final double kReverseSpeed = -0.8;
    }

    private final RollerModule m_leftModule;
    private final RollerModule m_rightModule;

    public Roller() {
        m_leftModule =
                new RollerModule(
                        new RollerModule.Config(
                                CANDevices.kLeftRollerMotorId,
                                CANDevices.kRioBus,
                                Roller.RollerConstants.kLeftInverted,
                                Roller.RollerConstants.kNeutralMode,
                                Roller.RollerConstants.kSupplyCurrentLimit,
                                Roller.RollerConstants.kStatorCurrentLimit));

        m_rightModule =
                new RollerModule(
                        new RollerModule.Config(
                                CANDevices.kRightRollerMotorId,
                                CANDevices.kRioBus,
                                Roller.RollerConstants.kRightInverted,
                                Roller.RollerConstants.kNeutralMode,
                                Roller.RollerConstants.kSupplyCurrentLimit,
                                Roller.RollerConstants.kStatorCurrentLimit));
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
        m_leftModule.setDutyCycle(Roller.RollerConstants.kForwardSpeed);
        m_rightModule.setDutyCycle(Roller.RollerConstants.kForwardSpeed);
    }

    /** Runs both rollers in reverse to unjam. */
    public void runReverse() {
        m_running = true;
        m_leftModule.setDutyCycle(Roller.RollerConstants.kReverseSpeed);
        m_rightModule.setDutyCycle(Roller.RollerConstants.kReverseSpeed);
    }

    /** Stops both roller motors. */
    public void stop() {
        m_running = false;
        m_leftModule.stop();
        m_rightModule.stop();
    }
}
