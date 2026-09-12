package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.junit.jupiter.api.Test;

class VisionFilterTest {

  private static Pose3d poseAt(double x, double y) {
    return new Pose3d(x, y, 0.0, new Rotation3d());
  }

  /** The filter only reads the translation, but a null rotation would NPE. */
  private static Pose2d currentAt(double x, double y) {
    return new Pose2d(x, y, Rotation2d.kZero);
  }

  @Test
  void acceptsGoodSingleTagEstimate() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.05, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void rejectsHighAmbiguitySingleTag() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.9, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void ignoresAmbiguityWhenMultipleTagsUsed() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 2, 0.9, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void rejectsPoseOutsideFieldBounds() {
    var d = VisionFilter.evaluate(poseAt(-3.0, 4.0), 2, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void rejectsImplausibleJumpFromCurrentEstimate() {
    var d = VisionFilter.evaluate(poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void rejectsEstimateWithNoTags() {
    var d = VisionFilter.evaluate(poseAt(4.0, 4.0), 0, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void farEstimateIsTrustedLessThanCloseOne() {
    var close = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 1.0, currentAt(4.0, 4.0), true, true);
    var far = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 4.0, currentAt(4.0, 4.0), true, true);
    assertTrue(far.stdDevs().get(0, 0) > close.stdDevs().get(0, 0));
  }

  @Test
  void multiTagIsTrustedMoreThanSingleTagAtSameDistance() {
    var single = VisionFilter.evaluate(poseAt(4.0, 4.0), 1, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    var multi = VisionFilter.evaluate(poseAt(4.0, 4.0), 3, 0.0, 2.0, currentAt(4.0, 4.0), true, true);
    assertTrue(multi.stdDevs().get(0, 0) < single.stdDevs().get(0, 0));
  }

  @Test
  void farAwayFirstEstimateIsAcceptedWhenNothingHasEverBeenAccepted() {
    // Phoenix odometry boots at (0, 0, 0deg). If the robot powers on far from
    // the origin, the jump gate must not block the very first estimate - that
    // would leave the fused pose stuck at the origin forever, since nothing
    // else can move it toward the truth.
    var d = VisionFilter.evaluate(
        poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(0.0, 0.0), false, true);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void sameFarAwayEstimateIsRejectedOnceSomethingHasBeenAccepted() {
    var d = VisionFilter.evaluate(
        poseAt(10.0, 4.0), 1, 0.0, 2.0, currentAt(0.0, 0.0), true, true);
    assertFalse(d.accept());
  }

  @Test
  void negativeCoordinatePoseIsAcceptedWhenFieldBoundsNotEnforced() {
    // The practice layout's origin is wherever the robot was parked for the
    // mapping session, not a field corner - legitimately negative coordinates
    // must not be rejected there.
    var d = VisionFilter.evaluate(
        poseAt(-3.0, -2.0), 1, 0.0, 2.0, currentAt(-3.0, -2.0), false, false);
    assertTrue(d.accept(), d.reason());
  }

  @Test
  void negativeCoordinatePoseIsStillRejectedWhenFieldBoundsEnforced() {
    var d = VisionFilter.evaluate(
        poseAt(-3.0, -2.0), 1, 0.0, 2.0, currentAt(-3.0, -2.0), false, true);
    assertFalse(d.accept());
  }
}
