package frc.robot.subsystems.localization.apriltag;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AprilTagLocalizationIOPolaris implements AprilTagLocalizationIO {
  private static final int cameraId = 0;
  private static final int cameraResolutionWidth = 1600;
  private static final int cameraResolutionHeight = 1200;
  private static final int cameraAutoExposure = 1;
  private static final int cameraExposure = 15;
  private static final int cameraGain = 25;
  private static final double tagSize = 0.1651;

  private final String name;
  private final Transform3d cameraToRobot;
  private final DoubleArraySubscriber observationSubscriber;
  private final IntegerSubscriber fpsSubscriber;

  public AprilTagLocalizationIOPolaris(String name, Transform3d robotToCamera) {
    this.name = name;
    this.cameraToRobot = robotToCamera.inverse();

    var table = NetworkTableInstance.getDefault().getTable(name);
    var config = table.getSubTable("config");
    config.getIntegerTopic("camera_id").publish().set(cameraId);
    config.getIntegerTopic("camera_resolution_width").publish().set(cameraResolutionWidth);
    config.getIntegerTopic("camera_resolution_height").publish().set(cameraResolutionHeight);
    config.getIntegerTopic("camera_auto_exposure").publish().set(cameraAutoExposure);
    config.getIntegerTopic("camera_exposure").publish().set(cameraExposure);
    config.getIntegerTopic("camera_gain").publish().set(cameraGain);
    config.getDoubleTopic("tag_size_m").publish().set(tagSize);

    try {
      config
          .getStringTopic("tag_layout")
          .publish()
          .set(new ObjectMapper().writeValueAsString(Constants.apriltagLayout));
    } catch (JsonProcessingException e) {
      throw new RuntimeException(e);
    }

    var outputTable = table.getSubTable("output");
    observationSubscriber =
        outputTable
            .getDoubleArrayTopic("observations")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
    fpsSubscriber = outputTable.getIntegerTopic("fps").subscribe(0, PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(AprilTagLocalizationInputs inputs) {
    var queue = observationSubscriber.readQueue();

    List<AprilTagPoseEstimate> validResults = new ArrayList<>();
    for (TimestampedDoubleArray frame : queue) {
      double[] data = frame.value;
      int numPoses = (int) data[0];

      if (numPoses == 0) {
        continue;
      }

      double timestamp = (frame.timestamp / 1000000.0);
      double ambiguity0 = data[1];
      Pose3d camPose0 =
          new Pose3d(
              new Translation3d(data[2], data[3], data[4]),
              new Rotation3d(new Quaternion(data[5], data[6], data[7], data[8])));
      Pose3d robotPose0 = camPose0.transformBy(cameraToRobot);

      Pose3d camPose1 = null;
      Pose3d robotPose1 = null;
      double ambiguity1 = 0.0;
      if (numPoses == 2) {
        ambiguity1 = data[9];
        camPose1 =
            new Pose3d(
                new Translation3d(data[10], data[11], data[12]),
                new Rotation3d(new Quaternion(data[13], data[14], data[15], data[16])));
        robotPose1 = camPose1.transformBy(cameraToRobot);
      }

      List<Integer> tagIDs = new ArrayList<>();
      double totalDistance0 = 0.0;
      double totalDistance1 = 0.0;
      int numTagsPresent = 0;
      for (int i = (numPoses == 1 ? 9 : 17); i < data.length; i++) {
        int id = (int) data[i];
        tagIDs.add(id);

        Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(id);
        if (tagPose.isPresent()) {
          totalDistance0 += tagPose.get().getTranslation().getDistance(camPose0.getTranslation());
          if (camPose1 != null) {
            totalDistance1 += tagPose.get().getTranslation().getDistance(camPose1.getTranslation());
          }
          numTagsPresent++;
        }
      }

      double avgDist0 = totalDistance0 / numTagsPresent;
      double avgDist1 = totalDistance1 / numTagsPresent;

      int[] tagIDsArr = tagIDs.stream().mapToInt(Integer::valueOf).toArray();

      validResults.add(
          new AprilTagPoseEstimate(
              robotPose0,
              ambiguity0,
              avgDist0,
              robotPose1,
              ambiguity1,
              avgDist1,
              timestamp - 0.02,
              tagIDsArr));
    }

    if (validResults.isEmpty()) {
      inputs.estimates = new ArrayList<>();
    } else {
      inputs.estimates = List.of(validResults.get(validResults.size() - 1));
    }

    var fpsQueue = fpsSubscriber.readQueue();
    if (fpsQueue.length > 0) {
      var last = fpsQueue[fpsQueue.length - 1];
      inputs.fps = (int) last.value;
      inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
    }
  }

  @Override
  public String getCameraName() {
    return name;
  }
}
