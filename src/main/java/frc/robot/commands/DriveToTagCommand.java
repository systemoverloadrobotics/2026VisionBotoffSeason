package frc.robot.commands;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.TargetSolution;
import frc.robot.util.CsvLogger;
import frc.robot.util.ProgressWatchdog;
import frc.robot.util.SettleDetector;
import java.util.Optional;
import java.util.function.IntSupplier;

/**
 * Drives to a standoff distance from a chosen tag while facing it.
 *
 * This is the stand-in for taking a shot. It consumes exactly the two numbers
 * a ShootCommand will consume - distance and bearing, from TargetSolution -
 * so mounting the shooter adds a consumer rather than changing this pipeline.
 *
 * Works when the tag is NOT visible: the target position comes from the
 * AprilTag layout and the robot position from the fused pose estimate, so the
 * camera is not on the critical path. When the tag IS visible, vision
 * measurements sharpen the fused pose continuously through the Kalman filter -
 * no mode change, no discontinuity.
 */
public class DriveToTagCommand extends Command {

  private final VisionSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private final IntSupplier m_tagIdSupplier;

  private final PIDController m_rangeController = new PIDController(
      AimConstants.kRangeP, AimConstants.kRangeI, AimConstants.kRangeD);

  private final SettleDetector m_settle =
      new SettleDetector(AimConstants.kSettledLoopsRequired);
  private final ProgressWatchdog m_watchdog = new ProgressWatchdog(
      AimConstants.kStallTimeoutSeconds,
      AimConstants.kStallProgressMeters,
      Math.toRadians(AimConstants.kStallProgressDegrees));

  private int m_tagId;
  private Translation2d m_target;
  private boolean m_hasTarget;
  private boolean m_settled;
  private boolean m_stalled;
  private boolean m_visionCorroborated;

  private static final double kDebugPeriodSeconds = 1.0;
  private final Timer m_debugTimer = new Timer();
  private CsvLogger m_csv;

  public DriveToTagCommand(
      VisionSwerveDrivetrain drivetrain, VisionSubsystem vision, IntSupplier tagIdSupplier) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_tagIdSupplier = tagIdSupplier;
    m_rangeController.setTolerance(AimConstants.kDistanceToleranceMeters);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_tagId = m_tagIdSupplier.getAsInt();
    Optional<Translation2d> target = m_vision.getTargetPosition(m_tagId);
    m_hasTarget = target.isPresent();
    m_target = target.orElse(Translation2d.kZero);

    m_rangeController.reset();
    m_settle.reset();
    m_watchdog.reset(Timer.getFPGATimestamp());
    m_settled = false;
    m_stalled = false;
    m_visionCorroborated = false;
    m_debugTimer.restart();

    m_csv = new CsvLogger("drive-to-tag",
        "timeSeconds", "tagId", "targetX", "targetY", "robotX", "robotY",
        "headingDeg", "bearingDeg", "headingErrorDeg", "distanceMeters", "rangeErrorMeters",
        "vxMps", "vyMps", "yawRateDegPerSec", "tagVisible", "settledLoops");

    if (!m_hasTarget) {
      System.out.printf(
          "DriveToTag: tag %d is not in the loaded layout - not moving%n", m_tagId);
    } else {
      System.out.printf("DriveToTag: tag %d, logging to %s%n", m_tagId, m_csv.getPath());
    }
  }

  @Override
  public void execute() {
    if (!m_hasTarget) {
      return;
    }

    Pose2d robotPose = m_drivetrain.getRobotPose();
    TargetSolution solution = TargetSolution.solve(robotPose, m_target);
    Rotation2d headingError = solution.headingError(robotPose.getRotation());
    double headingErrorRad = Math.abs(headingError.getRadians());
    double rangeError = solution.rangeErrorMeters(AimConstants.kStandoffMeters);
    double yawRate = m_drivetrain.getYawRateDegPerSec();

    // Speed along the line to the target. Positive rangeError means too far,
    // so drive toward it; negative means too close, so back off.
    double speed = MathUtil.clamp(
        m_rangeController.calculate(solution.distanceMeters(), AimConstants.kStandoffMeters),
        -AimConstants.kMaxApproachSpeedMps, AimConstants.kMaxApproachSpeedMps);

    // The controller drives distance DOWN toward the standoff, so its output is
    // negative when the robot is too far. Negate so positive speed means
    // "move toward the target".
    Translation2d velocity = solution.unitVectorToTarget().times(-speed);

    boolean withinRange = Math.abs(rangeError) <= AimConstants.kDistanceToleranceMeters;
    boolean aimed = headingErrorRad <= Math.toRadians(AimConstants.kHeadingToleranceDegrees);
    // This command controls TRANSLATION, not just heading, so "nearly stopped"
    // must require the chassis to have essentially stopped moving through the
    // field as well as stopped rotating - otherwise arrival can be declared
    // while the robot is still coasting through the tolerance band, exactly
    // the failure SettleDetector exists to prevent (the prototype cut motors
    // mid-motion and overshot by 14 degrees).
    boolean nearlyStopped = Math.abs(yawRate) < AimConstants.kSettledRateDegPerSec
        && m_drivetrain.getTranslationalSpeedMps() < AimConstants.kSettledSpeedMetersPerSec;
    m_settled = m_settle.update(withinRange && aimed, nearlyStopped);

    Optional<Rotation2d> liveBearing = m_vision.getLiveBearingToTag(m_tagId);
    if (m_settled && liveBearing.isPresent()
        && Math.abs(liveBearing.get().getDegrees()) <= AimConstants.kHeadingToleranceDegrees) {
      m_visionCorroborated = true;
    }

    // Progress on EITHER metric resets the stall timer - turning toward the
    // target legitimately makes no distance progress for a while.
    m_stalled = m_watchdog.update(
        Timer.getFPGATimestamp(), Math.abs(rangeError), headingErrorRad);

    m_drivetrain.driveFacingAngle(velocity.getX(), velocity.getY(), solution.bearing());

    m_csv.logRow(Timer.getFPGATimestamp(), m_tagId, m_target.getX(), m_target.getY(),
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(),
        solution.bearing().getDegrees(), headingError.getDegrees(),
        solution.distanceMeters(), rangeError,
        velocity.getX(), velocity.getY(), yawRate,
        liveBearing.isPresent(), m_settle.consecutiveLoops());

    if (m_debugTimer.advanceIfElapsed(kDebugPeriodSeconds)) {
      System.out.printf(
          "DriveToTag: tag=%d dist=%.2fm rangeErr=%.2fm headingErr=%.1fdeg "
              + "v=(%.2f, %.2f)m/s tagVisible=%s%n",
          m_tagId, solution.distanceMeters(), rangeError, headingError.getDegrees(),
          velocity.getX(), velocity.getY(), liveBearing.isPresent());

      DogLog.log("DriveToTag/DistanceMeters", solution.distanceMeters());
      DogLog.log("DriveToTag/RangeErrorMeters", rangeError);
      DogLog.log("DriveToTag/HeadingErrorDeg", headingError.getDegrees());
      DogLog.log("DriveToTag/TagVisible", liveBearing.isPresent());
    }
  }

  @Override
  public boolean isFinished() {
    return !m_hasTarget || m_stalled || m_settled;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
    m_csv.close();

    double bestRange = m_watchdog.bestSeen(0);
    double bestHeadingDeg = Math.toDegrees(m_watchdog.bestSeen(1));
    if (!m_hasTarget) {
      System.out.println("DriveToTag RESULT: NOT RUN - tag not in the layout");
    } else if (interrupted) {
      System.out.printf("DriveToTag RESULT: INTERRUPTED (closest %.2fm)%n", bestRange);
    } else if (m_visionCorroborated) {
      System.out.printf(
          "DriveToTag RESULT: ARRIVED, CAMERA-VERIFIED (range err %.2fm, heading err %.2fdeg)%n",
          bestRange, bestHeadingDeg);
    } else if (m_settled) {
      System.out.printf(
          "DriveToTag RESULT: ARRIVED ON ODOMETRY ONLY, NOT camera-verified "
              + "(range err %.2fm, heading err %.2fdeg)%n", bestRange, bestHeadingDeg);
    } else if (m_stalled) {
      System.out.printf(
          "DriveToTag RESULT: STALLED after %.1fs (closest range err %.2fm, "
              + "heading err %.2fdeg)%n",
          AimConstants.kStallTimeoutSeconds, bestRange, bestHeadingDeg);
    }
    DogLog.log("DriveToTag/LastResultCameraVerified", m_visionCorroborated);
  }
}
