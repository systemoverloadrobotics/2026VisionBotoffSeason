package frc.robot.subsystems;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.targeting.TagFieldLayoutLoader;
import frc.robot.targeting.VisionFilter;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The only class that talks to PhotonVision.
 *
 * Two jobs, kept separate on purpose:
 *
 * 1. LOCALIZATION (continuous, every loop): turn visible tags into a field
 *    pose and fuse it into the drivetrain's pose estimator. Nothing triggers
 *    this; it simply always runs.
 *
 * 2. RAW QUERIES (on demand): which tags are visible, what is the live bearing
 *    to one of them. Used ONLY to corroborate a targeting solution, never to
 *    steer the robot. Control always runs on the fused pose - the XRP
 *    prototype hard-switched control between vision and odometry, and the
 *    discontinuity at each switch was a real, repeatable bug.
 */
public class VisionSubsystem extends SubsystemBase {

  private final PhotonCamera m_camera = new PhotonCamera(VisionConstants.kCameraName);
  private final TagFieldLayoutLoader.LoadResult m_loadResult = TagFieldLayoutLoader.load();
  private final AprilTagFieldLayout m_layout = m_loadResult.layout();
  private final PhotonPoseEstimator m_poseEstimator;
  private final VisionSwerveDrivetrain m_drivetrain;

  private PhotonPipelineResult m_latestResult;
  private double m_lastResultTimestamp = -1.0;
  private double m_lastAcceptedTimestamp = -1.0;
  private double m_firstEnabledTimestamp = -1.0;
  private boolean m_warnedNoCamera;
  private int m_rejectedCount;

  public VisionSubsystem(VisionSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    // WHY THE STRATEGY DEPENDS ON WHICH LAYOUT IS LOADED:
    //
    // Multi-tag PnP is solved on the COPROCESSOR, against the field layout
    // uploaded to PhotonVision's web UI - NOT against the layout this robot
    // code loaded. PhotonPoseEstimator only uses our layout for an origin
    // transform on that path (see estimateCoprocMultiTagPose); the tag
    // positions come from the Orange Pi. The single-tag path is the opposite:
    // it calls fieldTags.getTagPose(id) against OUR layout.
    //
    // So with a hand-placed practice layout, multi-tag would happily report
    // poses derived from where those tag IDs sit on the real FRC field -
    // confidently wrong, by however far the practice space differs. It gets
    // worse from there: VisionFilter trusts multi-tag results 4x MORE than
    // single-tag ones, the ambiguity gate does not apply to them, and the
    // field-bounds gate is off while the practice layout is in use.
    //
    // Degrade to single-tag-but-correct rather than multi-tag-but-wrong. To
    // get multi-tag accuracy in the practice space, upload the same
    // practice-layout.json to the PhotonVision web UI and this guard stops
    // applying on its own once kUsePracticeLayout goes false for competition.
    PoseStrategy primaryStrategy = FieldConstants.kUsePracticeLayout
        ? PoseStrategy.LOWEST_AMBIGUITY
        : PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    m_poseEstimator = new PhotonPoseEstimator(
        m_layout, primaryStrategy, VisionConstants.kRobotToCamera);
    // Tags scattered around a practice space usually give ONE tag at a time,
    // so the fallback is the common path here, not the exception.
    m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    System.out.printf(
        "Vision: layout=%s strategy=%s%s%n",
        FieldConstants.kUsePracticeLayout ? "PRACTICE" : "OFFICIAL",
        primaryStrategy,
        m_loadResult.usedFallback() ? " (LAYOUT FALLBACK - targeting disabled)" : "");
  }

  @Override
  public void periodic() {
    // Drain every frame queued since the last loop. Each one is a separate
    // measurement at its own capture time, so all of them are worth fusing -
    // unlike the XRP version, which kept only the newest.
    List<PhotonPipelineResult> results = m_camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      m_latestResult = results.get(results.size() - 1);
      m_lastResultTimestamp = Timer.getFPGATimestamp();
    }

    for (PhotonPipelineResult result : results) {
      processResult(result);
    }

    checkCameraHealth();
    logTelemetry();
  }

  private void processResult(PhotonPipelineResult result) {
    Optional<org.photonvision.EstimatedRobotPose> estimate = m_poseEstimator.update(result);
    if (estimate.isEmpty()) {
      return;
    }
    var est = estimate.get();

    int tagCount = est.targetsUsed.size();
    double worstAmbiguity = 0.0;
    double totalDistance = 0.0;
    for (PhotonTrackedTarget target : est.targetsUsed) {
      worstAmbiguity = Math.max(worstAmbiguity, target.getPoseAmbiguity());
      totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
    }
    double avgDistance = tagCount > 0 ? totalDistance / tagCount : 0.0;

    VisionFilter.Decision decision = VisionFilter.evaluate(
        est.estimatedPose, tagCount, worstAmbiguity, avgDistance, m_drivetrain.getRobotPose(),
        m_lastAcceptedTimestamp >= 0, !FieldConstants.kUsePracticeLayout);

    if (!decision.accept()) {
      m_rejectedCount++;
      DogLog.log("Vision/LastRejectReason", decision.reason());
      return;
    }

    // CommandSwerveDrivetrain's override already converts this FPGA timestamp
    // into Phoenix's timebase, and rejects it if it falls outside the
    // estimator's history buffer.
    m_drivetrain.addVisionMeasurement(
        est.estimatedPose.toPose2d(), est.timestampSeconds, decision.stdDevs());
    m_lastAcceptedTimestamp = est.timestampSeconds;
  }

  /**
   * A PhotonCamera constructed with a name that does not match the
   * PhotonVision configuration does not throw - it silently yields zero
   * results forever, which is indistinguishable from "no tag in view". This
   * is the loud startup check that makes that case debuggable.
   */
  private void checkCameraHealth() {
    if (m_warnedNoCamera || !DriverStation.isEnabled()) {
      return;
    }
    double now = Timer.getFPGATimestamp();
    if (m_firstEnabledTimestamp < 0) {
      m_firstEnabledTimestamp = now;
    }
    if (m_lastResultTimestamp < 0
        && now - m_firstEnabledTimestamp > VisionConstants.kCameraTimeoutSeconds) {
      m_warnedNoCamera = true;
      DriverStation.reportError(
          "No result from PhotonVision camera '" + VisionConstants.kCameraName
              + "' after " + VisionConstants.kCameraTimeoutSeconds + "s. Check that the name "
              + "matches the PhotonVision web UI exactly, and that the coprocessor is up.",
          false);
    }
  }

  private void logTelemetry() {
    DogLog.log("Vision/Healthy", isHealthy());
    DogLog.log("Vision/HasTarget", hasTarget());
    DogLog.log("Vision/VisibleTagIds", getVisibleTagIds());
    DogLog.log("Vision/RejectedCount", m_rejectedCount);
    DogLog.log("Vision/SecondsSinceAccepted",
        m_lastAcceptedTimestamp < 0 ? -1.0 : Timer.getFPGATimestamp() - m_lastAcceptedTimestamp);
    Pose2d pose = m_drivetrain.getRobotPose();
    DogLog.log("Drivetrain/RobotX", pose.getX());
    DogLog.log("Drivetrain/RobotY", pose.getY());
    DogLog.log("Drivetrain/RobotHeadingDeg", pose.getRotation().getDegrees());
    DogLog.log("Drivetrain/YawRateDegPerSec", m_drivetrain.getYawRateDegPerSec());
  }

  public AprilTagFieldLayout getLayout() {
    return m_layout;
  }

  /**
   * Where a tag sits on the field, flattened to 2D. Empty if not in the
   * layout - OR if the practice layout was requested but a fallback to the
   * official layout occurred. Without this second check, targeting would stay
   * armed and happily find, say, tag 16 at its official FIELD position - a
   * real coordinate several meters away, in a practice room where that tag
   * does not exist - and drive toward it at up to kMaxApproachSpeedMps. Fail
   * closed instead: commands already treat an empty target as "not in the
   * layout" and finish with zero motion, so this routes into tested behavior.
   */
  public Optional<Translation2d> getTargetPosition(int tagId) {
    if (m_loadResult.usedFallback()) {
      return Optional.empty();
    }
    return m_layout.getTagPose(tagId).map(pose -> pose.toPose2d().getTranslation());
  }

  /**
   * True only when a recent, healthy camera frame has targets. isHealthy() is
   * required, not optional: m_latestResult is set once per frame and never
   * cleared, so without this check a frozen camera or coprocessor would leave
   * hasTarget() returning true forever off a dead frame. That would let
   * AimAtTagCommand and DriveToTagCommand report CAMERA-VERIFIED off a stale
   * image, and - worse - would let MapVisibleTagsCommand record tag positions
   * as the CURRENT robot pose composed with an OLD camera frame, silently
   * corrupting the practice layout that everything else depends on.
   */
  public boolean hasTarget() {
    return isHealthy() && m_latestResult != null && m_latestResult.hasTargets();
  }

  public boolean isHealthy() {
    return m_lastResultTimestamp >= 0
        && Timer.getFPGATimestamp() - m_lastResultTimestamp < VisionConstants.kCameraTimeoutSeconds;
  }

  public double getLastAcceptedTimestamp() {
    return m_lastAcceptedTimestamp;
  }

  public int[] getVisibleTagIds() {
    if (!hasTarget()) {
      return new int[0];
    }
    List<PhotonTrackedTarget> targets = m_latestResult.getTargets();
    int[] ids = new int[targets.size()];
    for (int i = 0; i < targets.size(); i++) {
      ids[i] = targets.get(i).getFiducialId();
    }
    return ids;
  }

  /**
   * Live robot-relative bearing to a specific tag, straight from the current
   * camera frame. Used ONLY to corroborate an arrival, never to steer.
   * Empty when that tag is not currently visible.
   */
  public Optional<Rotation2d> getLiveBearingToTag(int tagId) {
    if (!hasTarget()) {
      return Optional.empty();
    }
    for (PhotonTrackedTarget target : m_latestResult.getTargets()) {
      if (target.getFiducialId() == tagId) {
        Transform3d robotToTarget =
            VisionConstants.kRobotToCamera.plus(target.getBestCameraToTarget());
        return Optional.of(robotToTarget.getTranslation().toTranslation2d().getAngle());
      }
    }
    return Optional.empty();
  }

  /**
   * Full robot-relative 3D transform to a specific visible tag, camera mount
   * offset included. Used by the layout mapper, which needs the tag's
   * rotation as well as its position.
   */
  public Optional<Transform3d> getRobotToTagTransform(int tagId) {
    if (!hasTarget()) {
      return Optional.empty();
    }
    for (PhotonTrackedTarget target : m_latestResult.getTargets()) {
      if (target.getFiducialId() == tagId) {
        return Optional.of(
            VisionConstants.kRobotToCamera.plus(target.getBestCameraToTarget()));
      }
    }
    return Optional.empty();
  }
}
