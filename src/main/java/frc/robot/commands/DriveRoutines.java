package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RoutineConstants;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import java.util.function.BooleanSupplier;

/**
 * Drivebase verification routines. No camera, no AprilTag layout, no shooter.
 *
 * Each routine returns to its own starting point, so the closing error is
 * measurable against a tape mark on the floor.
 *
 * WHAT THE PRINTED ERROR DOES AND DOES NOT TELL YOU. These routines close the
 * loop on odometry, so a robot with slipping wheels or a wrong wheel diameter
 * will still report arriving back at its origin - it is grading its own
 * homework. The printed number only proves the control loop converged. The
 * number that matters is the physical distance from the tape mark, and the GAP
 * between the two is the odometry error you are hunting.
 *
 * Which routine to run when:
 *
 *   OUT-AND-BACK    one axis, no rotation. The simplest thing that can fail.
 *   SQUARE (HOLD)   both axes, still no rotation. Isolates translation and
 *                   wheel odometry - a bad close here is module offsets or
 *                   wheel scaling, not the gyro.
 *   SQUARE (TURNS)  both axes plus four 90-degree turns. If this closes worse
 *                   than SQUARE (HOLD), the difference is the gyro and the
 *                   rotational half of the kinematics.
 *
 * Running HOLD and TURNS back to back is the fastest way to isolate a gyro
 * problem from a module problem.
 */
public final class DriveRoutines {

  private DriveRoutines() {}

  /** Forward one leg, then back to the start. Heading held throughout. */
  public static Command outAndBack(VisionSwerveDrivetrain drivetrain, BooleanSupplier abort) {
    double d = RoutineConstants.kLegMeters;
    Origin origin = new Origin();
    return routine("OUT-AND-BACK", drivetrain, origin, abort,
        leg(drivetrain, origin, d, 0.0, 0.0, "out"),
        leg(drivetrain, origin, 0.0, 0.0, 0.0, "back"));
  }

  /**
   * A square traversed without ever rotating - the robot strafes each side.
   * Corners are (d,0), (d,d), (0,d), (0,0) relative to the origin.
   */
  public static Command squareHoldingHeading(
      VisionSwerveDrivetrain drivetrain, BooleanSupplier abort) {
    double d = RoutineConstants.kLegMeters;
    Origin origin = new Origin();
    return routine("SQUARE-HOLD", drivetrain, origin, abort,
        leg(drivetrain, origin, d, 0.0, 0.0, "hold-1"),
        leg(drivetrain, origin, d, d, 0.0, "hold-2"),
        leg(drivetrain, origin, 0.0, d, 0.0, "hold-3"),
        leg(drivetrain, origin, 0.0, 0.0, 0.0, "hold-4"));
  }

  /**
   * The same four corners, but the robot turns 90 degrees counter-clockwise at
   * each one, so every side is driven "forward" in its own robot frame - the
   * way a tank drive would have to do it.
   *
   * The turns are separate legs with a zero offset, which DriveRelativeCommand
   * treats as a pure rotation. Headings run 0, 90, 180, 270 and then back to 0;
   * Rotation2d normalises 270 to -90, so the last turn is the short way
   * forward (+90) rather than unwinding three quarters of a turn backwards.
   */
  public static Command squareWithTurns(
      VisionSwerveDrivetrain drivetrain, BooleanSupplier abort) {
    double d = RoutineConstants.kLegMeters;
    Origin origin = new Origin();
    return routine("SQUARE-TURNS", drivetrain, origin, abort,
        leg(drivetrain, origin, d, 0.0, 0.0, "turn-side-1"),
        leg(drivetrain, origin, d, 0.0, 90.0, "turn-corner-1"),
        leg(drivetrain, origin, d, d, 90.0, "turn-side-2"),
        leg(drivetrain, origin, d, d, 180.0, "turn-corner-2"),
        leg(drivetrain, origin, 0.0, d, 180.0, "turn-side-3"),
        leg(drivetrain, origin, 0.0, d, 270.0, "turn-corner-3"),
        leg(drivetrain, origin, 0.0, 0.0, 270.0, "turn-side-4"),
        leg(drivetrain, origin, 0.0, 0.0, 0.0, "turn-corner-4"));
  }

  // -----------------------------------------------------------------
  // Plumbing
  // -----------------------------------------------------------------

  /**
   * Holds the pose the routine started from, so every leg can target an offset
   * from the SAME point rather than from wherever the previous leg stopped.
   * Mutable and captured once per routine run - see DriveRelativeCommand's
   * class comment for why that distinction decides what the closing error
   * actually measures.
   */
  private static final class Origin {
    private Pose2d pose = Pose2d.kZero;
  }

  private static Command leg(
      VisionSwerveDrivetrain drivetrain, Origin origin,
      double xMeters, double yMeters, double headingDegrees, String name) {
    return new DriveRelativeCommand(
        drivetrain,
        () -> origin.pose,
        new Translation2d(xMeters, yMeters),
        Rotation2d.fromDegrees(headingDegrees),
        name)
        // Per-leg hard timeout. The stall watchdog catches a jammed robot; this
        // catches one hunting forever just outside tolerance, which would
        // otherwise hold the drivetrain for the rest of the match.
        .withTimeout(RoutineConstants.kLegTimeoutSeconds);
  }

  private static Command routine(
      String name, VisionSwerveDrivetrain drivetrain, Origin origin,
      BooleanSupplier abort, Command... legs) {

    Command body = Commands.sequence(
        Commands.runOnce(() -> {
          origin.pose = drivetrain.getRobotPose();
          System.out.printf(
              "%n=== Routine %s START at (%.3f, %.3f, %.1fdeg) ===%n"
                  + "    Mark the floor now - the printed closing error only proves the%n"
                  + "    control loop converged. The tape measure is the real result.%n",
              name, origin.pose.getX(), origin.pose.getY(),
              origin.pose.getRotation().getDegrees());
        }),
        Commands.sequence(legs));

    // Driver abort: moving either stick past a deadband ends the routine, so
    // the robot can be taken back without reaching for disable. Checked as a
    // race rather than inside each leg, so it applies between legs too.
    Command withAbort = body.until(abort);

    return withAbort
        .finallyDo(interrupted -> {
          drivetrain.stopDrive();
          Pose2d end = drivetrain.getRobotPose();
          Translation2d error = end.getTranslation().minus(origin.pose.getTranslation());
          double headingErrorDeg =
              end.getRotation().minus(origin.pose.getRotation()).getDegrees();
          System.out.printf(
              "=== Routine %s %s ===%n"
                  + "    start (%.3f, %.3f, %.1fdeg)  end (%.3f, %.3f, %.1fdeg)%n"
                  + "    closing error %.1f cm, %.1f deg (odometry's own opinion)%n"
                  + "    Now measure the robot against the floor mark - that is the%n"
                  + "    number that says whether odometry is telling the truth.%n%n",
              name, interrupted ? "ABORTED" : "COMPLETE",
              origin.pose.getX(), origin.pose.getY(), origin.pose.getRotation().getDegrees(),
              end.getX(), end.getY(), end.getRotation().getDegrees(),
              error.getNorm() * 100.0, headingErrorDeg);
        })
        .withName("Routine-" + name);
  }
}
