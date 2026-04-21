package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem — wraps a single PhotonVision camera for both AprilTag pose estimation and
 * target tracking.
 *
 * <p>Uses the PhotonVision 2026 API:
 *
 * <ul>
 *   <li>{@code getLatestResult()} is replaced by {@code getAllUnreadResults()}, which returns all
 *       frames since the last call and must be called exactly once per robot loop.
 *   <li>The 3-argument {@code PhotonPoseEstimator} constructor is replaced by the 2-argument form;
 *       strategy is selected by calling the appropriate estimation method directly.
 *   <li>Pose estimation calls {@code estimateCoprocMultiTagPose()} for multi-tag results and {@code
 *       estimateLowestAmbiguityPose()} for single-tag fallback.
 * </ul>
 *
 * <p>Pose estimates are not applied here — Swerve calls {@link #getEstimatedRobotPose(Pose2d)} each
 * loop and passes valid measurements to {@code addVisionMeasurement()}.
 */
public class Vision extends SubsystemBase {

    /**
     * Vision constants for PhotonVision pose estimation and target tracking.
     *
     * <p>A single camera handles both AprilTag pose estimation (fed into SwerveDrivePoseEstimator)
     * and target tracking (used for auto-aim). The robot-to-camera transform must be measured
     * carefully — even small errors in the transform will cause pose estimation drift.
     */
    public static final class VisionConstants {
        // Camera name must match the name configured in the PhotonVision UI.
        public static final String kCameraName = "photonvision"; // TODO: update to match UI

        // Robot-to-camera transform — describes where the camera is mounted
        // relative to the robot center. Measure from robot center (floor level)
        // to camera lens.
        // TODO: Measure all three values precisely on the physical robot.
        // Translation: forward(+X), left(+Y), up(+Z) in meters
        // Rotation: roll, pitch, yaw in radians
        public static final Transform3d kRobotToCamera =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(
                                        12.0), // forward from robot center — placeholder
                                Units.inchesToMeters(0.0), // left/right offset — placeholder
                                Units.inchesToMeters(24.0)), // height from floor — placeholder
                        new Rotation3d(
                                0.0, // roll — typically 0
                                Math.toRadians(
                                        -30.0), // pitch — tilt down toward targets, placeholder
                                0.0)); // yaw — 0 if camera faces forward

        // AprilTag field layout — used by PoseEstimationStrategy to compute
        // robot pose from tag detections. Load from the WPILib field layout
        // for the current game.
        public static final AprilTagFieldLayout kFieldLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Maximum ambiguity ratio — discard pose estimates above this threshold.
        // Lower = stricter filtering. 0.2 is a reasonable starting point.
        public static final double kMaxAmbiguity = 0.2;

        // Maximum distance from current estimated pose to accept a vision
        // measurement. Rejects wildly wrong estimates caused by tag misreads.
        public static final double kMaxPoseJumpMeters = 1.0;

        // ── Auto-Aim — Hub field positions ────────────────────────────────────────
        // Hub center coordinates in meters, using the WPILib blue-alliance-origin
        // field coordinate system (origin = blue alliance wall, left corner).
        // TODO: Look up exact values from the 2026 field dimension drawings.
        // The field is ~16.54m long x ~8.07m wide.
        // Blue hub is on the blue alliance side; red hub is on the red alliance side.
        public static final Translation2d kBlueHubCenter =
                new Translation2d(
                        Units.inchesToMeters(297.0), Units.inchesToMeters(161.6)); // placeholder
        public static final Translation2d kRedHubCenter =
                new Translation2d(
                        Units.inchesToMeters(354.2), Units.inchesToMeters(161.6)); // placeholder

        // Yaw tolerance — robot is considered aligned within this many degrees.
        // TODO: Tune empirically — tighter = more accurate shots, longer alignment time.
        public static final double kAutoAimToleranceDegrees = 2.0;

        // Auto-aim PID gains — output is rotation rate in rad/s.
        // TODO: Tune kP empirically. Start ~0.05 and increase until the robot
        // snaps to heading without oscillating.
        public static final double kAutoAimKP = 0.05;
        public static final double kAutoAimKI = 0.0;
        public static final double kAutoAimKD = 0.0;
    }

    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;

    // Cache of all unread pipeline results from the current loop.
    // Populated once per loop in periodic() — getAllUnreadResults() drains
    // the camera's internal FIFO queue and must only be called once per loop.
    private List<PhotonPipelineResult> m_unreadResults = List.of();

    public Vision() {
        m_camera = new PhotonCamera(Vision.VisionConstants.kCameraName);

        // 2-argument constructor — strategy is now selected by which estimation
        // method is called rather than being set at construction time.
        m_poseEstimator =
                new PhotonPoseEstimator(
                        Vision.VisionConstants.kFieldLayout, Vision.VisionConstants.kRobotToCamera);
    }

    @Override
    public void periodic() {
        // Drain the camera's result queue exactly once per loop.
        // getAllUnreadResults() clears the internal FIFO (max 20 entries) so
        // calling it more than once per loop would return empty results.
        m_unreadResults = m_camera.getAllUnreadResults();

        SmartDashboard.putBoolean("Vision/CameraConnected", m_camera.isConnected());
        SmartDashboard.putBoolean("Vision/HasTargets", hasTargets());
    }

    // ── Pose Estimation ───────────────────────────────────────────────────────

    /**
     * Returns the latest valid estimated robot pose from all unread camera results.
     *
     * <p>Iterates all unread results rather than only the most recent one — this ensures no frames
     * are skipped when the camera runs faster than the robot loop (e.g. 30 fps camera on a 50 Hz
     * robot loop).
     *
     * <p>Multi-tag results use {@code estimateCoprocMultiTagPose()} for best accuracy. Single-tag
     * results fall back to {@code estimateLowestAmbiguityPose()} and are filtered by ambiguity
     * threshold.
     *
     * <p>Call from {@link frc.robot.subsystems.swerve.Swerve#periodic()} and pass the result to
     * {@code addVisionMeasurement()} if present.
     *
     * @param currentEstimate Current pose estimate used to reject implausible measurements
     * @return Optional estimated pose and timestamp, or empty if no valid measurement available
     */
    public Optional<EstimatedRobotPose> getEstimatedRobotPose(Pose2d currentEstimate) {
        Optional<EstimatedRobotPose> bestResult = Optional.empty();

        for (var pipelineResult : m_unreadResults) {
            if (!pipelineResult.hasTargets()) {
                continue;
            }

            // Use multi-tag coprocessor solve when multiple tags are visible —
            // more accurate and doesn't require per-tag ambiguity filtering.
            // Fall back to lowest-ambiguity single-tag estimate otherwise.
            Optional<EstimatedRobotPose> result =
                    pipelineResult.getMultiTagResult().isPresent()
                            ? m_poseEstimator.estimateCoprocMultiTagPose(pipelineResult)
                            : m_poseEstimator.estimateLowestAmbiguityPose(pipelineResult);

            if (result.isEmpty()) {
                continue;
            }

            // Reject high-ambiguity single-tag estimates — they are often wrong.
            // Multi-tag estimates don't have meaningful per-tag ambiguity so this
            // filter only applies when exactly one tag is used.
            if (result.get().targetsUsed.size() == 1) {
                double ambiguity = result.get().targetsUsed.get(0).getPoseAmbiguity();
                if (ambiguity > Vision.VisionConstants.kMaxAmbiguity) {
                    continue;
                }
            }

            // Reject estimates that jump too far from the current known pose —
            // likely caused by a tag misread or camera glitch.
            double jumpMeters =
                    result.get()
                            .estimatedPose
                            .toPose2d()
                            .getTranslation()
                            .getDistance(currentEstimate.getTranslation());
            if (jumpMeters > Vision.VisionConstants.kMaxPoseJumpMeters) {
                continue;
            }

            // Keep iterating — later results in the list are more recent.
            bestResult = result;
        }

        return bestResult;
    }

    // ── Target Tracking ───────────────────────────────────────────────────────

    /** Returns true if any unread result from this loop contains at least one target. */
    public boolean hasTargets() {
        return m_unreadResults.stream().anyMatch(r -> r.hasTargets());
    }

    /**
     * Returns the best target from the most recent unread result, or empty if no targets are
     * visible. Used by auto-aim commands to get yaw for alignment.
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (m_unreadResults.isEmpty()) {
            return Optional.empty();
        }
        // Use the last entry — getAllUnreadResults() returns results in
        // chronological order so the last element is the most recent.
        var latest = m_unreadResults.get(m_unreadResults.size() - 1);
        if (!latest.hasTargets()) {
            return Optional.empty();
        }
        return Optional.of(latest.getBestTarget());
    }

    /**
     * Returns the yaw angle to the best visible target in degrees. Positive = target is to the left
     * of camera center. Returns 0.0 if no target is visible.
     */
    public double getTargetYaw() {
        return getBestTarget().map(PhotonTrackedTarget::getYaw).orElse(0.0);
    }

    /**
     * Returns true if the robot is aligned to the best visible target within the given yaw
     * tolerance.
     *
     * @param toleranceDegrees Acceptable yaw error in degrees
     */
    public boolean isAlignedToTarget(double toleranceDegrees) {
        return hasTargets() && Math.abs(getTargetYaw()) <= toleranceDegrees;
    }
}
