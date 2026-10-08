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
import frc.robot.Constants.RoutineConstants;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import frc.robot.targeting.TargetSolution;
import frc.robot.util.CsvLogger;
import frc.robot.util.ProgressWatchdog;
import frc.robot.util.SettleDetector;
import java.util.function.Supplier;

/**
 * Drives to a field-relative offset from a routine's ORIGIN, holding a heading.
 *
 * One leg of a drivebase verification routine (see DriveRoutines). Needs no
 * camera and no AprilTag layout - it reads the fused pose and nothing else,
 * which is exactly what makes it usable before a camera is mounted.
 *
 * OFFSETS ARE FROM THE ROUTINE'S ORIGIN, NOT FROM THE PREVIOUS LEG'S END.
 * That is the whole point. If each leg were relative to wherever the last one
 * happened to stop, every leg's control error would stack, and the closing
 * error at the end would be a sum of control errors rather than a measurement
 * of odometry. Targeting each leg from the shared origin means the final leg
 * aims at the true starting point and absorbs whatever the earlier legs left
 * behind.
 *
 * A ZERO OFFSET MAKES THIS A PURE TURN. The turning square uses that - it
 * needs no second command class, and the turn inherits the same settle, stall,
 * and logging behavior as the translations.
 */
public class DriveRelativeCommand extends Command {

  private final VisionSwerveDrivetrain m_drivetrain;
  private final Supplier<Pose2d> m_originSupplier;
  private final Translation2d m_fieldOffset;
  private final Rotation2d m_targetHeading;
  private final String m_legName;

  private final PIDController m_rangeController = new PIDController(
      AimConstants.kRangeP, AimConstants.kRangeI, AimConstants.kRangeD);

  private final SettleDetector m_settle =
      new SettleDetector(AimConstants.kSettledLoopsRequired);
  private final ProgressWatchdog m_watchdog = new ProgressWatchdog(
      AimConstants.kStallTimeoutSeconds,
      AimConstants.kStallProgressMeters,
      Math.toRadians(AimConstants.kStallProgressDegrees));

  private Translation2d m_target;
  private boolean m_settled;
  private boolean m_stalled;

  private static final double kDebugPeriodSeconds = 0.5;
  private final Timer m_debugTimer = new Timer();
  private CsvLogger m_csv;

  /**
   * @param originSupplier the routine's starting pose, captured once when the
   *                       routine began - NOT this leg's starting pose
   * @param fieldOffset    where to end up, relative to that origin, in field
   *                       coordinates
   * @param targetHeading  heading to hold throughout, absolute (field) frame
   * @param legName        appears in the console and the CSV filename
   */
  public DriveRelativeCommand(
      VisionSwerveDrivetrain drivetrain,
      Supplier<Pose2d> originSupplier,
      Translation2d fieldOffset,
      Rotation2d targetHeading,
      String legName) {
    m_drivetrain = drivetrain;
    m_originSupplier = originSupplier;
    m_fieldOffset = fieldOffset;
    m_targetHeading = targetHeading;
    m_legName = legName;
    m_rangeController.setTolerance(RoutineConstants.kPositionToleranceMeters);
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    m_target = m_originSupplier.get().getTranslation().plus(m_fieldOffset);

    m_rangeController.reset();
    m_settle.reset();
    m_watchdog.reset(Timer.getFPGATimestamp());
    m_settled = false;
    m_stalled = false;
    m_debugTimer.restart();

    m_csv = new CsvLogger("leg-" + m_legName,
        "timeSeconds", "targetX", "targetY", "robotX", "robotY", "headingDeg",
        "targetHeadingDeg", "headingErrorDeg", "distanceMeters",
        "vxMps", "vyMps", "yawRateDegPerSec", "translSpeedMps", "settledLoops");

    System.out.printf(
        "Leg %s: to (%.3f, %.3f) heading %.0fdeg, logging to %s%n",
        m_legName, m_target.getX(), m_target.getY(), m_targetHeading.getDegrees(),
        m_csv.getPath());
  }

  @Override
  public void execute() {
    Pose2d robotPose = m_drivetrain.getRobotPose();
    TargetSolution solution = TargetSolution.solve(robotPose, m_target);

    // Heading error is against the FIXED target heading, not the bearing to the
    // target. This command holds an attitude while translating; it does not
    // point at where it is going. A zero offset therefore turns in place.
    Rotation2d headingError = m_targetHeading.minus(robotPose.getRotation());
    double headingErrorRad = Math.abs(headingError.getRadians());

    double distance = solution.distanceMeters();
    double yawRate = m_drivetrain.getYawRateDegPerSec();
    double translSpeed = m_drivetrain.getTranslationalSpeedMps();

    // Same sign convention as DriveToTagCommand: the controller drives distance
    // DOWN toward zero, so its output is negative while the robot is short of
    // the target; negate so positive means "move toward it".
    double speed = MathUtil.clamp(
        m_rangeController.calculate(distance, 0.0),
        -RoutineConstants.kMaxSpeedMps, RoutineConstants.kMaxSpeedMps);
    Translation2d velocity = solution.unitVectorToTarget().times(-speed);

    boolean inPosition = distance <= RoutineConstants.kPositionToleranceMeters;
    boolean aimed = headingErrorRad <= Math.toRadians(AimConstants.kHeadingToleranceDegrees);
    // Both axes must be stopped, not just the one this leg mostly moves - a
    // pure turn still has to stop translating, and a pure translation still
    // has to stop rotating.
    boolean nearlyStopped = Math.abs(yawRate) < AimConstants.kSettledRateDegPerSec
        && translSpeed < AimConstants.kSettledSpeedMetersPerSec;
    m_settled = m_settle.update(inPosition && aimed, nearlyStopped);

    m_stalled = m_watchdog.update(Timer.getFPGATimestamp(), distance, headingErrorRad);

    m_drivetrain.driveRoutineFacingAngle(velocity.getX(), velocity.getY(), m_targetHeading);

    m_csv.logRow(Timer.getFPGATimestamp(), m_target.getX(), m_target.getY(),
        robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees(),
        m_targetHeading.getDegrees(), headingError.getDegrees(), distance,
        velocity.getX(), velocity.getY(), yawRate, translSpeed,
        m_settle.consecutiveLoops());

    if (m_debugTimer.advanceIfElapsed(kDebugPeriodSeconds)) {
      System.out.printf(
          "Leg %s: dist=%.3fm headingErr=%.1fdeg v=(%.2f, %.2f)m/s%n",
          m_legName, distance, headingError.getDegrees(),
          velocity.getX(), velocity.getY());
      DogLog.log("Routine/LegDistanceMeters", distance);
      DogLog.log("Routine/LegHeadingErrorDeg", headingError.getDegrees());
    }
  }

  @Override
  public boolean isFinished() {
    return m_settled || m_stalled;
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stopDrive();
    m_csv.close();

    Pose2d pose = m_drivetrain.getRobotPose();
    double remaining = pose.getTranslation().getDistance(m_target);
    if (interrupted) {
      System.out.printf("Leg %s: INTERRUPTED %.3fm short%n", m_legName, remaining);
    } else if (m_stalled) {
      System.out.printf(
          "Leg %s: STALLED after %.1fs with %.3fm remaining%n",
          m_legName, AimConstants.kStallTimeoutSeconds, remaining);
    } else {
      System.out.printf("Leg %s: done, %.3fm from leg target%n", m_legName, remaining);
    }
  }
}
