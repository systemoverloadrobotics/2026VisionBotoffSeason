package frc.robot.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.Constants.AimConstants;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Checks the real src/main/deploy/practice-layout.json - the same file that is
 * uploaded to PhotonVision. A mistyped quaternion here would turn a tag to face
 * the wall, and every pose solved from it would be confidently wrong.
 */
class PracticeLayoutFileTest {

  private static AprilTagFieldLayout load() throws IOException {
    return new AprilTagFieldLayout(Path.of("src/main/deploy/practice-layout.json"));
  }

  @Test
  void hasTagsOneThroughSix() throws IOException {
    List<Integer> ids = load().getTags().stream().map(t -> t.ID).sorted().toList();
    assertEquals(List.of(1, 2, 3, 4, 5, 6), ids);
  }

  @Test
  void everyTagFacesIntoTheField() throws IOException {
    AprilTagFieldLayout layout = load();
    for (AprilTag tag : layout.getTags()) {
      // A tag's pose points OUT of its printed face, so 1 m along its own X
      // axis must land inside the field rectangle.
      Pose2d inFront = tag.pose.toPose2d().transformBy(new Transform2d(1.0, 0.0, Rotation2d.kZero));
      assertTrue(inFront.getX() > 0 && inFront.getX() < layout.getFieldLength()
              && inFront.getY() > 0 && inFront.getY() < layout.getFieldWidth(),
          "tag " + tag.ID + " faces out of the field: " + inFront);
    }
  }

  @Test
  void defaultTargetTagIsInTheLayout() throws IOException {
    assertTrue(load().getTagPose(AimConstants.kDefaultTargetTagId).isPresent());
  }

  @Test
  void tagsSitNearCameraHeight() throws IOException {
    for (AprilTag tag : load().getTags()) {
      assertEquals(0.40, tag.pose.getZ(), 1e-9, "tag " + tag.ID);
    }
  }
}
