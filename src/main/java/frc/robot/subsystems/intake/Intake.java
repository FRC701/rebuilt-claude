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
 *
 * <p>If the arm ever jerks hard on deploy and you're worried about mechanism stress, the right fix
 * is adding a kS feedforward to the Position request (not switching to Motion Magic) — it gives the
 * motor a small initial kick without the full trapezoidal profile overhead.
 *
 * <p>Tuning Order
 *
 * <p>1. Set kP the same on both slots (moderate value, e.g. 0.5) 2. Increase kG on slot 1 in small
 * steps until the arm lifts reliably without drifting back 3. Fine-tune kP on each slot
 * independently for crisp position holding 4. Only split kP between slots if the slop behavior
 * still warrants it after kG is dialed in — you may find equal kP works fine once gravity is
 * compensated
 *
 * <p>Tuning for bounce
 *
 * <p>1. Get basic deploy/retract working and PID tuned first 2. Set kBounceAmplitudeRotations = 1.0
 * and kBounceHalfPeriodSeconds = 0.3 as a starting point 3. Increase amplitude if jams still occur,
 * decrease if the arm is moving too aggressively 4. Adjust half-period — faster (0.2s) for more
 * aggressive bounce, slower (0.5s) for gentler
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
        m_deployModule.setPosition(IntakeConstants.kDeployedPosition, 0);
    }

    /**
     * Commands the arm to the current bounce target position.
     *
     * <p>Called every loop from IntakeDeploy once the arm is deployed. Uses slot 0 (deploy gains)
     * since the arm stays near the deployed position throughout the oscillation.
     *
     * @param elapsedSeconds time since bouncing started, from the command's Timer
     */
    public void deployBounce(double elapsedSeconds) {
        m_deployModule.setPosition(getBouncePosition(elapsedSeconds), 0);
    }

    /** Commands the arm to the fully retracted position. */
    public void retract() {
        m_deployModule.setPosition(IntakeConstants.kRetractedPosition, 1);
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

    /**
     * Returns the current bounce target position based on which half of the oscillation cycle the
     * timer is in.
     *
     * <p>Kept in Intake rather than the command so the subsystem owns all position knowledge — the
     * command just calls deployBounce() each loop.
     */
    public double getBouncePosition(double elapsedSeconds) {
        // Integer division of elapsed time by half-period determines which
        // half-cycle we are in. Even = deployed position, odd = bounce peak.
        int halfCycles = (int) (elapsedSeconds / IntakeConstants.kBounceHalfPeriodSeconds);
        return (halfCycles % 2 == 0)
                ? IntakeConstants.kDeployedPosition
                : IntakeConstants.kDeployedPosition + IntakeConstants.kBounceAmplitudeRotations;
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
