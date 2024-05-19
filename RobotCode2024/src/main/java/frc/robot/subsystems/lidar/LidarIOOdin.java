package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LidarIOOdin implements LidarIO {
  private final String deviceName;
  private final Transform3d robotToLidar;
  private final DoubleArraySubscriber pointsSubscriber;

  public LidarIOOdin(
      String deviceName, double angleThreshMin, double angleThreshMax, Transform3d robotToLidar) {
    this.deviceName = deviceName;
    this.robotToLidar = robotToLidar;

    var table = NetworkTableInstance.getDefault().getTable(deviceName);
    var config = table.getSubTable("config");

    config.getDoubleTopic("angle_thresh_min").publish().set(angleThreshMin);
    config.getDoubleTopic("angle_thresh_max").publish().set(angleThreshMax);
    config.getDoubleTopic("max_distance").publish().set(Constants.Lidar.maxDistance);

    var outputTable = table.getSubTable("output");
    pointsSubscriber =
        outputTable
            .getDoubleArrayTopic("points")
            .subscribe(
                new double[] {}, PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(LidarInputs inputs) {
    var queue = pointsSubscriber.readQueue();

    if (queue.length > 0) {
      TimestampedDoubleArray last = queue[queue.length - 1];

      Pose3d robotPose = new Pose3d(RobotContainer.swerve.getPose());

      Translation3d[] points = new Translation3d[(int) Math.floor(last.value.length / 2.0)];
      for (int i = 0; i < points.length; i++) {
        int idx = i * 2;

        Translation3d sensorRelative = new Translation3d(last.value[idx], last.value[idx + 1], 0.0);
        Translation3d robotRelative =
            sensorRelative.rotateBy(robotToLidar.getRotation()).plus(robotToLidar.getTranslation());
        Translation3d fieldRelative =
            robotRelative.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
        points[i] = fieldRelative;
      }
      inputs.fieldPoints = points;
    }
  }

  @Override
  public String getName() {
    return deviceName;
  }
}
