package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

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

    /**
     * Intake hardware constants.
     *
     * <p>Deploy motor uses position control so the arm holds at a known angle. Roller motor uses
     * open-loop duty cycle — no need for closed-loop on a simple intake roller. Both motors are on
     * the RIO bus because the CANivore is already carrying the swerve hardware and keeping
     * high-frequency devices together there is cleaner.
     */
    public static final class IntakeConstants {

        // --- Motor config ---
        // Brake on deploy keeps the arm from back-driving under gravity/impact.
        // Brake on roller ensures it stops quickly when command ends.
        public static final NeutralModeValue kDeployNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue kRollerNeutralMode = NeutralModeValue.Brake;

        // Inversion — verify on first power-up; positive output should deploy outward.
        public static final boolean kDeployInverted = false;
        public static final boolean kRollerInverted = false;

        // --- Current limits ---
        // Supply limit protects wiring; stator limit prevents mechanism damage.
        public static final double kDeploySupplyCurrentLimit = 40.0; // amps
        public static final double kDeployStatorCurrentLimit = 60.0; // amps
        public static final double kRollerSupplyCurrentLimit = 30.0; // amps
        public static final double kRollerStatorCurrentLimit = 40.0; // amps

        // --- Deploy positions (rotations of the motor shaft) ---
        // TODO: Measure empirically — jog motor and read sensor position in Tuner X.
        public static final double kDeployedPosition = 10.0; // rotations — placeholder
        public static final double kRetractedPosition = 0.0; // rotations — home/zero

        // --- Deploy PID gains (Slot 0) ---
        // Higher kP pushes through linkage slop quickly on the way out.
        // TODO: Tune with manual sweep. Start ~0.5, increase until arm moves
        // crisply without oscillating.
        //
        // --- Gravity feedforward ---
        // kG is a constant voltage (volts) added to the output to counteract gravity.
        // Tune by finding the minimum voltage that holds the arm stationary at the
        // hardest point against gravity (usually mid-travel). Start at 0.0 and
        // increase in small steps (~0.1V) until the arm holds without drifting.
        // TODO: Tune empirically — kG will likely be higher for retract (lifting)
        // than deploy (lowering) if the arm geometry changes mechanical advantage
        // across the range of motion.
        public static final double kDeployKP = 0.5;
        public static final double kDeployKI = 0.0; // I term rarely needed for position
        public static final double kDeployKD = 0.0;
        public static final double kDeployKG = 0.0; // volts — placeholder

        // --- Retract PID gains (Slot 1) ---
        // Softer kP on retract — mechanism is being pulled back and slop is
        // taken up in the opposite direction, so a hard snap is less useful
        // and risks slamming the arm against the retracted hard stop.
        // TODO: Start lower than kDeployKP (~0.3) and tune separately.
        public static final double kRetractKP = 0.3;
        public static final double kRetractKI = 0.0;
        public static final double kRetractKD = 0.0;
        public static final double kRetractKG = 0.0; // volts — placeholder

        // Position tolerance — arm is considered "at target" within this many rotations.
        public static final double kDeployToleranceRotations = 0.5;

        // --- Bounce constants ---
        // Bouncing oscillates the deploy arm between two positions while intaking
        // to prevent game pieces from jamming. Timer-based switching is used so
        // the arm moves predictably regardless of PID settle time.
        //
        // kBounceAmplitudeRotations defines how far above kDeployedPosition the arm
        // travels on the upstroke — keep small enough that the arm stays clearly
        // deployed but large enough to dislodge jams.
        // TODO: Tune empirically — start small (~1.0 rotation) and increase if
        // bouncing is not effective.
        public static final double kBounceAmplitudeRotations = 1.0; // rotations above deployed
        public static final double kBounceHalfPeriodSeconds = 0.3; // seconds per half-cycle

        // --- Roller speeds ---
        // Duty cycle (-1.0 to 1.0). Positive = intaking, negative = ejecting.
        public static final double kRollerForwardSpeed = 0.8;
        public static final double kRollerReverseSpeed = -0.8;
    }

    private final IntakeModule m_deployModule;
    private final IntakeModule m_rollerModule;

    public Intake() {
        // Deploy motor config — position control, brake, heavier current limits.
        m_deployModule =
                new IntakeModule(
                        new IntakeModule.Config(
                                CANDevices.kIntakeDeployId,
                                CANDevices.kRioBus,
                                Intake.IntakeConstants.kDeployInverted,
                                Intake.IntakeConstants.kDeployNeutralMode,
                                Intake.IntakeConstants.kDeploySupplyCurrentLimit,
                                Intake.IntakeConstants.kDeployStatorCurrentLimit));

        // Roller motor config — open-loop duty cycle, brake.
        m_rollerModule =
                new IntakeModule(
                        new IntakeModule.Config(
                                CANDevices.kIntakeRollerId,
                                CANDevices.kRioBus,
                                Intake.IntakeConstants.kRollerInverted,
                                Intake.IntakeConstants.kRollerNeutralMode,
                                Intake.IntakeConstants.kRollerSupplyCurrentLimit,
                                Intake.IntakeConstants.kRollerStatorCurrentLimit));

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
        m_deployModule.setPosition(Intake.IntakeConstants.kDeployedPosition, 0);
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
        m_deployModule.setPosition(Intake.IntakeConstants.kRetractedPosition, 1);
    }

    /**
     * Returns true when the arm is within tolerance of the deployed position. Used by commands to
     * know when it's safe to run the rollers.
     */
    public boolean isDeployed() {
        return Math.abs(m_deployModule.getPosition() - Intake.IntakeConstants.kDeployedPosition)
                < Intake.IntakeConstants.kDeployToleranceRotations;
    }

    /** Returns true when the arm is within tolerance of the retracted position. */
    public boolean isRetracted() {
        return Math.abs(m_deployModule.getPosition() - Intake.IntakeConstants.kRetractedPosition)
                < Intake.IntakeConstants.kDeployToleranceRotations;
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
        int halfCycles = (int) (elapsedSeconds / Intake.IntakeConstants.kBounceHalfPeriodSeconds);
        return (halfCycles % 2 == 0)
                ? Intake.IntakeConstants.kDeployedPosition
                : Intake.IntakeConstants.kDeployedPosition
                        + Intake.IntakeConstants.kBounceAmplitudeRotations;
    }

    // --- Roller control ---

    /** Runs rollers inward to intake game pieces. */
    public void runRollerForward() {
        m_rollerModule.setDutyCycle(Intake.IntakeConstants.kRollerForwardSpeed);
    }

    /** Runs rollers outward to eject game pieces. */
    public void runRollerReverse() {
        m_rollerModule.setDutyCycle(Intake.IntakeConstants.kRollerReverseSpeed);
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
