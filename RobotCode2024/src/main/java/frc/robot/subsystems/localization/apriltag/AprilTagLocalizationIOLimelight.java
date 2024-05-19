package frc.robot.subsystems.localization.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.TimestampedString;
import frc.lib.vision.LimelightHelpers;
import java.util.ArrayList;
import java.util.List;

public class AprilTagLocalizationIOLimelight implements AprilTagLocalizationIO {
  private final String limelightName;
  private final StringSubscriber jsonDumpSub;

  public AprilTagLocalizationIOLimelight(String limelightName) {
    this.limelightName = limelightName;
    this.jsonDumpSub =
        NetworkTableInstance.getDefault()
            .getTable(limelightName)
            .getStringTopic("json")
            .subscribe("", PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  @Override
  public String getCameraName() {
    return limelightName;
  }

  @Override
  public void updateInputs(AprilTagLocalizationInputs inputs) {
    TimestampedString[] queue = jsonDumpSub.readQueue();

    List<AprilTagPoseEstimate> validResults = new ArrayList<>();
    for (TimestampedString timestampedString : queue) {
      double timestamp = timestampedString.timestamp / 1000000.0;
      LimelightHelpers.Results results =
          LimelightHelpers.parseJsonDump(timestampedString.value).targetingResults;

      if (results.targets_Fiducials.length == 0) {
        continue;
      }

      double latencyMS = results.latency_capture + results.latency_pipeline;
      // Subtract the latency from the NT timestamp
      timestamp -= (latencyMS / 1000.0);

      Pose3d botPoseBlue = results.getBotPose3d_wpiBlue();
      double averageTagDistance = 0;
      int[] tagIDs = new int[results.targets_Fiducials.length];
      for (int i = 0; i < results.targets_Fiducials.length; i++) {
        tagIDs[i] = (int) Math.round(results.targets_Fiducials[i].fiducialID);
        averageTagDistance +=
            results.targets_Fiducials[i].getTargetPose_CameraSpace().getTranslation().getNorm();
      }
      averageTagDistance /= tagIDs.length;

      validResults.add(
          new AprilTagPoseEstimate(
              botPoseBlue, 0.0, averageTagDistance, null, 0.0, 0.0, timestamp, tagIDs));
    }

    inputs.estimates = validResults;
  }
}
