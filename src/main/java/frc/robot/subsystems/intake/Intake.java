package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

/**
 * Intake subsystem — coordinates the deploy arm and roller.
 *
 * <p>Follows the project's Shooter/ShooterModule pattern: this class handles high-level
 * coordination (seeding the encoder on init, publishing telemetry) while {@link IntakeModule} owns
 * the TalonFX hardware details.
 *
 * <p>The deploy arm uses relative position control. The encoder is seeded to zero at robot init,
 * which assumes the arm starts in the retracted position. If the robot could be powered on in any
 * position, an absolute encoder or limit switch would be needed instead.
 */
public class Intake extends SubsystemBase {

    private final IntakeModule m_deployModule;
    private final IntakeModule m_rollerModule;

    public Intake() {
        // Deploy motor config — position control, brake, heavier current limits.
        m_deployModule =
                new IntakeModule(
                        new IntakeModule.Config(
                                CANDevices.kIntakeDeployId,
                                CANDevices.kRioBus,
                                IntakeConstants.kDeployInverted,
                                IntakeConstants.kDeployNeutralMode,
                                IntakeConstants.kDeploySupplyCurrentLimit,
                                IntakeConstants.kDeployStatorCurrentLimit));

        // Roller motor config — open-loop duty cycle, brake.
        m_rollerModule =
                new IntakeModule(
                        new IntakeModule.Config(
                                CANDevices.kIntakeRollerId,
                                CANDevices.kRioBus,
                                IntakeConstants.kRollerInverted,
                                IntakeConstants.kRollerNeutralMode,
                                IntakeConstants.kRollerSupplyCurrentLimit,
                                IntakeConstants.kRollerStatorCurrentLimit));

        // Seed the relative encoder to zero assuming the arm starts retracted.
        // Must happen after configuration is applied.
        m_deployModule.seedEncoder();
    }

    @Override
    public void periodic() {
        // Telemetry — useful during tuning and match-day debugging.
        SmartDashboard.putNumber("Intake/DeployPosition", m_deployModule.getPosition());
        SmartDashboard.putBoolean("Intake/IsDeployed", isDeployed());
        SmartDashboard.putBoolean("Intake/IsRetracted", isRetracted());
    }

    // --- Deploy control ---

    /** Commands the arm to the fully deployed position. */
    public void deploy() {
        m_deployModule.setPosition(IntakeConstants.kDeployedPosition);
    }

    /** Commands the arm to the fully retracted position. */
    public void retract() {
        m_deployModule.setPosition(IntakeConstants.kRetractedPosition);
    }

    /**
     * Returns true when the arm is within tolerance of the deployed position. Used by commands to
     * know when it's safe to run the rollers.
     */
    public boolean isDeployed() {
        return Math.abs(m_deployModule.getPosition() - IntakeConstants.kDeployedPosition)
                < IntakeConstants.kDeployToleranceRotations;
    }

    /** Returns true when the arm is within tolerance of the retracted position. */
    public boolean isRetracted() {
        return Math.abs(m_deployModule.getPosition() - IntakeConstants.kRetractedPosition)
                < IntakeConstants.kDeployToleranceRotations;
    }

    // --- Roller control ---

    /** Runs rollers inward to intake game pieces. */
    public void runRollerForward() {
        m_rollerModule.setDutyCycle(IntakeConstants.kRollerForwardSpeed);
    }

    /** Runs rollers outward to eject game pieces. */
    public void runRollerReverse() {
        m_rollerModule.setDutyCycle(IntakeConstants.kRollerReverseSpeed);
    }

    /** Stops the roller motor. */
    public void stopRoller() {
        m_rollerModule.stop();
    }

    /** Stops both motors immediately. */
    public void stopAll() {
        m_deployModule.stop();
        m_rollerModule.stop();
    }
}
