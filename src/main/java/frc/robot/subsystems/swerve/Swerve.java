package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Vision;

/*
 * A few design decisions worth noting:
 *
 * * SwerveDrivePoseEstimator is used instead of SwerveDriveOdometry — it's a
 *   drop-in replacement that also accepts vision measurements, so adding
 *   Limelight later requires no refactoring
 * * ChassisSpeeds.discretize() corrects for a known drift issue when rotating
 *   while translating — this is a WPILib best practice for swerve
 * * lockWheels() puts modules in an X-pattern to resist being pushed — useful
 *   at the end of auto or when defending
 * * addVisionMeasurement() is stubbed in and ready for when you add vision later
 *
 */

/*
 * Tuning Swerve
 *
 * - Get mechanical measurements in Constants.java
 * - Verify motor inversions and CANcoder offsets on the real robot
 * - Run SysId to get drive feedforward and PID gains
 * - Tune steer PID (usually just kP empirically)
 * - Tune PathPlanner gains last
 */

public class Swerve extends SubsystemBase {

    // ── Modules ───────────────────────────────────────────────────────────────
    private final SwerveModule m_frontLeft;
    private final SwerveModule m_frontRight;
    private final SwerveModule m_backLeft;
    private final SwerveModule m_backRight;
    private Vision m_vision = null;

    // ── Gyro ──────────────────────────────────────────────────────────────────
    private final Pigeon2 m_pigeon;

    // ── Odometry ──────────────────────────────────────────────────────────────
    // SwerveDrivePoseEstimator is preferred over SwerveDriveOdometry because
    // it supports vision measurement fusion (e.g. Limelight) later on.
    private final SwerveDrivePoseEstimator m_poseEstimator;

    // ── Field visualization (Shuffleboard) ────────────────────────────────────
    private final Field2d m_field = new Field2d();

    // Mutable holders for SysId measurements — reused each loop to avoid GC
    private final MutVoltage m_sysIdAppliedVoltage = Volts.mutable(0);
    private final MutDistance m_sysIdPosition = Meters.mutable(0);
    private final MutLinearVelocity m_sysIdVelocity = MetersPerSecond.mutable(0);

    private final SysIdRoutine m_sysIdRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(),
                    new SysIdRoutine.Mechanism(
                            // Drive all modules at the same voltage, wheels pointed straight
                            voltage -> {
                                for (SwerveModule module : getModulesArray()) {
                                    module.setDriveVoltage(voltage.in(Volts));
                                }
                            },
                            // Log drive position, velocity, and voltage for each module
                            log -> {
                                SwerveModule[] modules = getModulesArray();
                                String[] names = {
                                    "FrontLeft", "FrontRight", "BackLeft", "BackRight"
                                };
                                for (int i = 0; i < modules.length; i++) {
                                    log.motor(names[i])
                                            .voltage(
                                                    m_sysIdAppliedVoltage.mut_replace(
                                                            modules[i].getDriveVoltage(), Volts))
                                            .linearPosition(
                                                    m_sysIdPosition.mut_replace(
                                                            modules[i].getDrivePositionMeters(),
                                                            Meters))
                                            .linearVelocity(
                                                    m_sysIdVelocity.mut_replace(
                                                            modules[i].getDriveVelocityMPS(),
                                                            MetersPerSecond));
                                }
                            },
                            this));

    public Swerve() {
        // ── Instantiate modules ───────────────────────────────────────────────
        m_frontLeft =
                new SwerveModule(
                        "Front Left",
                        CANDevices.kFrontLeftDriveID,
                        CANDevices.kFrontLeftSteerID,
                        CANDevices.kFrontLeftCanCoderID,
                        SwerveConstants.kFrontLeftCanCoderOffset,
                        SwerveConstants.kFrontLeftDriveInvert);

        m_frontRight =
                new SwerveModule(
                        "Front Right",
                        CANDevices.kFrontRightDriveID,
                        CANDevices.kFrontRightSteerID,
                        CANDevices.kFrontRightCanCoderID,
                        SwerveConstants.kFrontRightCanCoderOffset,
                        SwerveConstants.kFrontRightDriveInvert);

        m_backLeft =
                new SwerveModule(
                        "Back Left",
                        CANDevices.kBackLeftDriveID,
                        CANDevices.kBackLeftSteerID,
                        CANDevices.kBackLeftCanCoderID,
                        SwerveConstants.kBackLeftCanCoderOffset,
                        SwerveConstants.kBackLeftDriveInvert);

        m_backRight =
                new SwerveModule(
                        "Back Right",
                        CANDevices.kBackRightDriveID,
                        CANDevices.kBackRightSteerID,
                        CANDevices.kBackRightCanCoderID,
                        SwerveConstants.kBackRightCanCoderOffset,
                        SwerveConstants.kBackRightDriveInvert);

        // ── Instantiate Pigeon 2 ──────────────────────────────────────────────
        m_pigeon = new Pigeon2(CANDevices.kPigeonID, CANDevices.kCanIvoreBus);

        // ── Reset drive encoders ──────────────────────────────────────────────
        resetDriveEncoders();

        // ── Instantiate pose estimator ────────────────────────────────────────
        m_poseEstimator =
                new SwerveDrivePoseEstimator(
                        SwerveConstants.kKinematics,
                        getGyroYaw(),
                        getModulePositions(),
                        new Pose2d());

        // ── Publish field to SmartDashboard ───────────────────────────────────
        SmartDashboard.putData("Field", m_field);
    }

    /**
     * Registers the vision subsystem for pose estimation fusion. Call this from RobotContainer
     * after both subsystems are constructed.
     *
     * @param vision The Vision subsystem
     */
    public void setVision(Vision vision) {
        m_vision = vision;
    }

    // ── Periodic ──────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        // Fuse vision measurements into the pose estimator if vision is available.
        // Vision is checked for null so Swerve works correctly even if vision
        // is disabled or not yet configured.
        if (m_vision != null) {
            m_vision.getEstimatedRobotPose(getPose())
                    .ifPresent(
                            estimate ->
                                    addVisionMeasurement(
                                            estimate.estimatedPose.toPose2d(),
                                            estimate.timestampSeconds));
        }

        // Update pose estimator with latest module positions and gyro angle
        m_poseEstimator.update(getGyroYaw(), getModulePositions());

        // Push robot pose to field visualization
        m_field.setRobotPose(getPose());

        // Publish telemetry to SmartDashboard
        publishTelemetry();
    }

    // ── Drive API ─────────────────────────────────────────────────────────────

    /**
     * Primary drive method. Accepts field-relative or robot-relative speeds.
     *
     * @param speeds Desired {@link ChassisSpeeds}
     * @param fieldRelative If true, speeds are field-relative (recommended for teleop)
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        ChassisSpeeds relativeSpeeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getGyroYaw())
                        : speeds;

        // Discretize to correct for drift caused by rotating while translating
        ChassisSpeeds discretized = ChassisSpeeds.discretize(relativeSpeeds, 0.02);

        SwerveModuleState[] desiredStates =
                SwerveConstants.kKinematics.toSwerveModuleStates(discretized);

        // Scale down all modules if any exceeds the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, SwerveConstants.kMaxDriveVelocityMPS);

        setModuleStates(desiredStates);
    }

    /**
     * Drives the robot using field-relative {@link ChassisSpeeds}. Convenience overload —
     * field-relative is the default for teleop.
     */
    public void drive(ChassisSpeeds speeds) {
        drive(speeds, true);
    }

    /** Stops all modules. */
    public void stop() {
        drive(new ChassisSpeeds(), false);
    }

    /**
     * Sets all modules to an X-pattern to resist pushing. Useful when stopped and don't want to be
     * moved.
     */
    public void lockWheels() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // ── Odometry & Pose ───────────────────────────────────────────────────────

    /** Returns the current estimated robot pose. */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the pose estimator to the given pose. Call this at the start of auto with the known
     * starting pose.
     *
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    /**
     * Adds a vision measurement to the pose estimator. Call this whenever your vision system (e.g.
     * Limelight) has a new pose.
     *
     * @param visionPose The pose measured by vision
     * @param timestampSeconds The FPGA timestamp of the measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
        m_poseEstimator.addVisionMeasurement(visionPose, timestampSeconds);
    }

    /** Returns the current robot-relative chassis speeds. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.kKinematics.toChassisSpeeds(getModuleStates());
    }

    // ── Gyro ──────────────────────────────────────────────────────────────────

    /** Returns the current yaw of the Pigeon 2 as a {@link Rotation2d}. */
    public Rotation2d getGyroYaw() {
        return m_pigeon.getRotation2d();
    }

    /**
     * Zeroes the gyro heading. Call this when the robot is facing away from you (field forward).
     */
    public void zeroGyro() {
        m_pigeon.setYaw(0.0);
    }

    // ── Module Helpers ────────────────────────────────────────────────────────

    /** Sets all four modules to the given states. */
    public void setModuleStates(SwerveModuleState[] states) {
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);
    }

    /** Returns the current states of all four modules. */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }

    /** Returns the current positions of all four modules. Used for odometry. */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    /** Resets all drive encoders to zero. */
    public void resetDriveEncoders() {
        m_frontLeft.resetDriveEncoder();
        m_frontRight.resetDriveEncoder();
        m_backLeft.resetDriveEncoder();
        m_backRight.resetDriveEncoder();
    }

    // ── Telemetry ─────────────────────────────────────────────────────────────

    private void publishTelemetry() {
        SmartDashboard.putNumber("Gyro/Yaw Degrees", getGyroYaw().getDegrees());

        SwerveModuleState[] states = getModuleStates();
        SmartDashboard.putNumber("Swerve/FL Speed MPS", states[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/FR Speed MPS", states[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/BL Speed MPS", states[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/BR Speed MPS", states[3].speedMetersPerSecond);
        SmartDashboard.putNumber("Swerve/FL Angle Degrees", states[0].angle.getDegrees());
        SmartDashboard.putNumber("Swerve/FR Angle Degrees", states[1].angle.getDegrees());
        SmartDashboard.putNumber("Swerve/BL Angle Degrees", states[2].angle.getDegrees());
        SmartDashboard.putNumber("Swerve/BR Angle Degrees", states[3].angle.getDegrees());

        Pose2d pose = getPose();
        SmartDashboard.putNumber("Odometry/X Meters", pose.getX());
        SmartDashboard.putNumber("Odometry/Y Meters", pose.getY());
        SmartDashboard.putNumber("Odometry/Heading Degrees", pose.getRotation().getDegrees());
    }

    /**
     * Returns a SysId quasistatic command for the drive motors. Run this slowly in both directions
     * to characterize kS and kV.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.quasistatic(direction);
    }

    /**
     * Returns a SysId dynamic command for the drive motors. Run this with a fast voltage ramp in
     * both directions to characterize kA.
     *
     * @param direction Forward or Reverse
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutine.dynamic(direction);
    }

    /** Returns all four modules as an array for iteration. */
    private SwerveModule[] getModulesArray() {
        return new SwerveModule[] {m_frontLeft, m_frontRight, m_backLeft, m_backRight};
    }
}
