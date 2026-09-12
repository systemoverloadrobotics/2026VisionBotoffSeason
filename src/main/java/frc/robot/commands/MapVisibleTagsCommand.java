package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSwerveDrivetrain;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Authors a practice-space AprilTag layout with the robot instead of a tape
 * measure.
 *
 * Usage:
 *   1. Park the robot at a measured spot and resetPose() to it. This is the
 *      ONLY pose reset in the whole project - everything else works in the
 *      live field frame.
 *   2. Drive around. Wherever tags are visible, press RECORD.
 *   3. Press WRITE. The JSON lands in the deploy directory of the running
 *      program; copy it into src/main/deploy/practice-layout.json.
 *
 * Accuracy compounds from that one measured starting pose plus odometry drift,
 * so keep mapping runs short and start near the tags.
 */
public final class MapVisibleTagsCommand {

  private MapVisibleTagsCommand() {}

  /** Tag ID to its field pose. Insertion-ordered so the JSON is readable. */
  private static final Map<Integer, Pose3d> s_mapped = new LinkedHashMap<>();

  /** Records every currently-visible tag's field pose. Bind to POV up. */
  public static Command record(VisionSwerveDrivetrain drivetrain, VisionSubsystem vision) {
    return Commands.runOnce(() -> {
      int[] ids = vision.getVisibleTagIds();
      if (ids.length == 0) {
        System.out.println("MapTags: no tags visible - nothing recorded");
        return;
      }
      Pose3d robotPose = new Pose3d(drivetrain.getRobotPose());
      for (int id : ids) {
        Transform3d robotToTag = vision.getRobotToTagTransform(id).orElse(null);
        if (robotToTag == null) {
          continue;
        }
        Pose3d tagPose = robotPose.transformBy(robotToTag);
        s_mapped.put(id, tagPose);
        System.out.printf(
            "MapTags: recorded tag %d at (%.3f, %.3f, %.3f) yaw=%.1fdeg%n",
            id, tagPose.getX(), tagPose.getY(), tagPose.getZ(),
            Math.toDegrees(tagPose.getRotation().getZ()));
      }
      System.out.printf("MapTags: %d tag(s) mapped so far%n", s_mapped.size());
    });
  }

  /** Serializes everything recorded so far. Bind to POV down. */
  public static Command write() {
    return Commands.runOnce(() -> {
      if (s_mapped.isEmpty()) {
        System.out.println("MapTags: nothing recorded - not writing a file");
        return;
      }
      List<AprilTag> tags = new ArrayList<>();
      s_mapped.forEach((id, pose) -> tags.add(new AprilTag(id, pose)));

      AprilTagFieldLayout layout = new AprilTagFieldLayout(
          tags, FieldConstants.kFieldLengthMeters, FieldConstants.kFieldWidthMeters);
      Path out = Filesystem.getDeployDirectory().toPath()
          .resolve(FieldConstants.kPracticeLayoutFile);
      try {
        layout.serialize(out);
        System.out.println("MapTags: wrote " + tags.size() + " tag(s) to " + out);
        System.out.println("MapTags: copy this file into src/main/deploy/ and redeploy");
      } catch (IOException e) {
        System.out.println("MapTags: FAILED to write " + out + ": " + e.getMessage());
      }
    });
  }

  /** Clears the in-memory map, so a bad run can be discarded without a reboot. */
  public static Command clear() {
    return Commands.runOnce(() -> {
      s_mapped.clear();
      System.out.println("MapTags: cleared");
    });
  }
}
