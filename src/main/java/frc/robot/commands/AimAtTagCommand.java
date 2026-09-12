package frc.robot.commands;

import dev.doglog.DogLog;
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
 * Rotates in place to face a chosen tag. No translation.
 *
 * Control runs on the FUSED pose, never on a raw camera bearing. The camera is
 * used only to corroborate the result - see the vision-verified logic below.
 * Hard-switching control onto a live camera bearing is what made the prototype
 * need several button presses to converge: the target heading jumped every time
 * the source flipped.
 */
public class AimAtTagCommand extends Command {

  private final VisionSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_vision;
  private final IntSupplier m_tagIdSupplier;

  private final SettleDetector m_settle =
      new SettleDetector(AimConstants.kSettledLoopsRequired);
  private final ProgressWatchdog m_watchdog = new ProgressWatchdog(
      AimConstants.kStallTimeoutSeconds,
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

  public AimAtTagCommand(
      VisionSwerveDrivetrain drivetrain, VisionSubsystem vision, IntSupplier tagIdSupplier) {
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_tagIdSupplier = tagIdSupplier;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_tagId = m_tagIdSupplier.getAsInt();
    Optional<Translation2d> target = m_vision.getTargetPosition(m_tagId);
    m_hasTarget = target.isPresent();
    m_target = target.orElse(Translation2d.kZero);

    m_settle.reset();
    m_watchdog.reset(Timer.getFPGATimestamp());
    m_settled = false;
    m_stalled = false;
    m_visionCorroborated = false;
    m_debugTimer.restart();

    m_csv = new CsvLogger("aim",
        "timeSeconds", "tagId", "targetX", "targetY", "robotX", "robotY",
        "headingDeg", "bearingDeg", "headingErrorDeg", "yawRateDegPerSec",
        "tagVisible", "liveBearingDeg", "settledLoops");

    if (!m_hasTarget) {
      System.out.printf(
          "Aim: tag %d is not in the loaded layout - nothing to aim at%n", m_tagId);
    } else {
      System.out.printf("Aim: tag %d, logging to %s%n", m_tagId, m_csv.getPath());
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
    double errorRad = Math.abs(headingError.getRadians());
    double yawRate = m_drivetrain.getYawRateDegPerSec();

    boolean withinTolerance = errorRad <= Math.toRadians(AimConstants.kHeadingToleranceDegrees);
    boolean nearlyStopped = Math.abs(yawRate) < AimConstants.kSettledRateDegPerSec;
    m_settled = m_settle.update(withinTolerance, nearlyStopped);

    // Corroboration, not control: if the tag is visible and its live bearing
    // agrees the robot is aimed, the result is camera-verified rather than
    // merely dead-reckoned. Settling without this is the silent-undershoot
    // case - odometry says aimed, nothing outside it agrees.
    Optional<Rotation2d> liveBearing = m_vision.getLiveBearingToTag(m_tagId);
    if (m_settled && liveBearing.isPresent()
        && Math.abs(liveBearing.get().getDegrees()) <= AimConstants.kHeadingToleranceDegrees) {
      m_visionCorroborated = true;
    }

    m_stalled = m_watchdog.update(Timer.getFPGATimestamp(), errorRad);

    // Rotation only. The heading controller inside the request produces the
    // rotation rate; zero translation keeps the robot in place.
    m_drivetrain.driveFacingAngle(0.0, 0.0, solution.bearing());

    m_csv.logRow(Timer.getFPGATimestamp(), m_tagId, m_target.getX(), m_target.getY(),
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(),
        solution.bearing().getDegrees(), headingError.getDegrees(), yawRate,
        liveBearing.isPresent(),
        liveBearing.map(Rotation2d::getDegrees).orElse(Double.NaN),
        m_settle.consecutiveLoops());

    if (m_debugTimer.advanceIfElapsed(kDebugPeriodSeconds)) {
      System.out.printf(
          "Aim: tag=%d bearing=%.1fdeg heading=%.1fdeg err=%.1fdeg yawRate=%.1fdeg/s "
              + "tagVisible=%s%n",
          m_tagId, solution.bearing().getDegrees(), robotPose.getRotation().getDegrees(),
          headingError.getDegrees(), yawRate, liveBearing.isPresent());

      DogLog.log("Aim/TagId", m_tagId);
      DogLog.log("Aim/BearingDeg", solution.bearing().getDegrees());
      DogLog.log("Aim/HeadingErrorDeg", headingError.getDegrees());
      DogLog.log("Aim/TagVisible", liveBearing.isPresent());
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

    double bestErrorDeg = Math.toDegrees(m_watchdog.bestSeen(0));
    if (!m_hasTarget) {
      System.out.println("Aim RESULT: NOT RUN - tag not in the layout");
    } else if (interrupted) {
      System.out.printf("Aim RESULT: INTERRUPTED (best error %.2fdeg)%n", bestErrorDeg);
    } else if (m_visionCorroborated) {
      System.out.printf("Aim RESULT: AIMED, CAMERA-VERIFIED (error %.2fdeg)%n", bestErrorDeg);
    } else if (m_settled) {
      System.out.printf(
          "Aim RESULT: AIMED ON ODOMETRY ONLY, NOT camera-verified (error %.2fdeg). "
              + "Do not trust this for a shot.%n", bestErrorDeg);
    } else if (m_stalled) {
      System.out.printf(
          "Aim RESULT: STALLED after %.1fs with no progress (best error %.2fdeg)%n",
          AimConstants.kStallTimeoutSeconds, bestErrorDeg);
    }
    DogLog.log("Aim/LastResultCameraVerified", m_visionCorroborated);
  }
}
