package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Every tuning number for the vision port, in one place. */
public final class Constants {

  private Constants() {}

  public static final class OperatorConstants {
    private OperatorConstants() {}
    public static final int kDriverControllerPort = 0;
  }

  public static final class FieldConstants {
    private FieldConstants() {}

    /** 2026 REBUILT field, from the official AprilTag layout JSON. */
    public static final double kFieldLengthMeters = 16.541;
    public static final double kFieldWidthMeters = 8.069;

    /** True = load src/main/deploy/practice-layout.json; false = official 2026 layout. */
    public static final boolean kUsePracticeLayout = true;
    public static final String kPracticeLayoutFile = "practice-layout.json";
  }

  public static final class VisionConstants {
    private VisionConstants() {}

    /**
     * Must match the camera name in the PhotonVision web UI EXACTLY.
     * A mismatch does not throw - PhotonCamera silently returns zero results
     * forever, indistinguishable from "no tag in view".
     */
    public static final String kCameraName = "arducam_camera";

    /**
     * Where the camera sits relative to robot center, in meters, plus its
     * rotation. PLACEHOLDER - measure the real mount (translation AND
     * rotation) before trusting any localization. Leaving the rotation at
     * identity is a known, specifically-documented mistake.
     */
    public static final Transform3d kRobotToCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(0.0), 0.0, Units.inchesToMeters(8.0)),
        new Rotation3d());

    /**
     * Reject single-tag solves above this pose ambiguity. 0.2 is the common
     * community starting point, NOT tuned for this camera/tag setup.
     * Multi-tag PnP solves report no meaningful ambiguity.
     */
    public static final double kMaxAmbiguity = 0.2;

    /** Reject an estimate this far from the current fused pose. */
    public static final double kMaxPoseJumpMeters = 1.0;

    /** Log loudly if no camera result has arrived this long after enable. */
    public static final double kCameraTimeoutSeconds = 5.0;

    /** Std dev scaling. Baseline is for a single tag at 1 m; grows with distance^2. */
    public static final double kSingleTagBaseStdDevMeters = 0.20;
    public static final double kMultiTagBaseStdDevMeters = 0.05;
    public static final double kThetaStdDevRadians = 0.5;
  }

  public static final class AimConstants {
    private AimConstants() {}

    public static final int kDefaultTargetTagId = 16;

    /** How close to stop from the tag. Stands in for shooting range. */
    public static final double kStandoffMeters = Units.inchesToMeters(36.0);
    public static final double kDistanceToleranceMeters = 0.05;

    public static final double kHeadingToleranceDegrees = 2.0;

    /**
     * A turn is not "done" just because error dipped under tolerance on one
     * loop - it must also have essentially STOPPED, for several consecutive
     * loops. Without this the motors cut mid-rotation and momentum carries
     * the robot past the target.
     */
    public static final double kSettledRateDegPerSec = 5.0;

    /**
     * DriveToTagCommand controls TRANSLATION, not just heading, so "settled"
     * must also mean the robot has essentially stopped moving through the
     * field, not merely stopped rotating. Without this, arrival can be
     * declared while the robot is still coasting through the tolerance band -
     * exactly the failure SettleDetector exists to prevent (the prototype cut
     * motors mid-motion and overshot by 14 degrees).
     */
    public static final double kSettledSpeedMetersPerSec = 0.05;

    public static final int kSettledLoopsRequired = 5;

    /** Give up if neither distance nor heading improves within this window. */
    public static final double kStallTimeoutSeconds = 3.0;
    public static final double kStallProgressMeters = 0.02;
    public static final double kStallProgressDegrees = 1.0;

    /**
     * Hard upper bound on any automated aiming/driving command, applied with
     * .withTimeout() at the binding.
     *
     * The stall watchdog above covers "stopped making progress", but a command
     * that requires the robot to SETTLE and never quite can would otherwise
     * hold the drivetrain and cost the driver control entirely. Requiring a
     * settle means also guaranteeing an exit.
     *
     * Derived, not guessed - it must be long enough for the worst case the
     * rate limit below allows to actually finish:
     *   180 deg turn / 25 deg/s rate limit   = 7.2s of pure rotation
     *   + settle time (kSettledLoopsRequired loops, negligible but nonzero)
     *   + translation margin for DriveToTagCommand, which turns AND
     *     translates inside the same budget                     ~= 7.8s
     *   total, rounded up with headroom                          = 15.0s
     * These two constants are COUPLED: raising kMaxAimRateRadPerSec shortens
     * the worst-case turn time and this timeout can shrink; lowering it (e.g.
     * once real camera latency is measured and the rate is tightened) makes
     * the worst-case turn take longer and this timeout must grow to match, or
     * large turns will time out and report INTERRUPTED no matter how well the
     * heading controller is tuned.
     */
    public static final double kCommandTimeoutSeconds = 15.0;

    /**
     * Max aiming turn rate. MUST be derived from measured pipeline latency,
     * not guessed: lag_degrees = turn_rate_deg_per_sec * latency_seconds.
     * Placeholder assumes ~40 ms latency and a ~1 deg lag budget.
     */
    public static final double kMaxAimRateRadPerSec = Math.toRadians(25.0);

    /** Max translation speed during an automated approach. */
    public static final double kMaxApproachSpeedMps = 1.5;

    // Heading controller, operating on radians -> rad/s. NOT tuned.
    public static final double kHeadingP = 4.0;
    public static final double kHeadingI = 0.0;
    public static final double kHeadingD = 0.0;

    // Range controller, operating on meters -> m/s. NOT tuned.
    public static final double kRangeP = 1.5;
    public static final double kRangeI = 0.0;
    public static final double kRangeD = 0.0;
  }

  /**
   * Drivebase verification routines - open-field test patterns that need no
   * camera and no AprilTag layout.
   *
   * These exist to answer one question: does the robot end up where odometry
   * says it does? Every routine returns to its own starting point, so the
   * closing error is measurable with a tape mark on the floor.
   *
   * IMPORTANT: the routines close the loop on odometry, so their PRINTED error
   * only proves the control loop converged - a robot with slipping wheels or a
   * wrong wheel diameter still reports arriving at (0,0). The number that
   * matters is the physical distance from the tape mark. The gap between the
   * two IS the odometry error.
   */
  public static final class RoutineConstants {
    private RoutineConstants() {}

    /** Side length for the out-and-back and square routines. */
    public static final double kLegMeters = 1.0;

    /**
     * Speed cap for automated test routines. Deliberately far below the
     * drivetrain's 4.58 m/s: these run in a shop with people nearby, and a
     * slower run also keeps wheel slip (which corrupts the very odometry being
     * measured) out of the result.
     */
    public static final double kMaxSpeedMps = 1.0;

    /** Rotation cap for the turning square's 90-degree corners. */
    public static final double kMaxTurnRateRadPerSec = Math.toRadians(90.0);

    /**
     * Per-leg hard timeout. A leg that cannot settle would otherwise hold the
     * drivetrain: the stall watchdog catches a jammed robot, but not one
     * hunting forever just outside tolerance.
     */
    public static final double kLegTimeoutSeconds = 6.0;

    /**
     * Tighter than AimConstants.kDistanceToleranceMeters (0.05). These routines
     * exist to MEASURE closing error, so settling at 5cm would put the
     * tolerance in the same range as the error being measured.
     */
    public static final double kPositionToleranceMeters = 0.02;

    /**
     * Stick deflection past which a running routine aborts, so the driver can
     * take the robot back without reaching for the disable button. Above any
     * plausible resting drift on a worn stick.
     */
    public static final double kAbortStickDeflection = 0.25;
  }
}
