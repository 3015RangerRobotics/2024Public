package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;

public class LidarIOSim implements LidarIO {
  private static final double resolution = 0.5;

  private final String deviceName;
  private final Transform3d robotToLidar;
  private final double angleThreshMin;
  private final double angleThreshMax;
  private final Timer updateTimer;
  private final double updatePeriod;

  public LidarIOSim(
      String deviceName,
      double angleThreshMin,
      double angleThreshMax,
      Transform3d robotToLidar,
      int updateHz) {
    this.deviceName = deviceName;
    this.robotToLidar = robotToLidar;
    this.angleThreshMin = angleThreshMin;
    this.angleThreshMax = angleThreshMax;
    this.updateTimer = new Timer();
    this.updatePeriod = 1.0 / updateHz;

    this.updateTimer.start();
  }

  @Override
  public void updateInputs(LidarInputs inputs) {
    if (!updateTimer.hasElapsed(updatePeriod)) {
      return;
    }

    updateTimer.reset();

    Pose3d robotPose = new Pose3d(RobotContainer.swerve.getPose());
    Pose2d sensorPose = robotPose.transformBy(robotToLidar).toPose2d();

    //    Pose2d defenseBotPose = RobotContainer.defenseBot.getPose();
    Pose2d defenseBotPose = new Pose2d();

    List<Translation2d> fieldPoints = new ArrayList<>();
    for (double theta = angleThreshMin; theta <= angleThreshMax; theta += resolution) {
      Rotation2d rot = Rotation2d.fromDegrees(theta).plus(sensorPose.getRotation());
      for (double d = 0.05; d <= Constants.Lidar.maxDistance; d += 0.05) {
        Translation2d testPointField = new Translation2d(d, rot).plus(sensorPose.getTranslation());

        if (testPointField.getX() <= 0.1
            || testPointField.getY() <= 0.1
            || testPointField.getX() >= Constants.fieldSize.getX() - 0.1
            || testPointField.getY() >= Constants.fieldSize.getY() - 0.1) {
          fieldPoints.add(testPointField);
          break;
        }

        Translation2d testPointDefBotRelative =
            testPointField
                .minus(defenseBotPose.getTranslation())
                .rotateBy(defenseBotPose.getRotation());
        if (Math.abs(testPointDefBotRelative.getX()) <= 0.45
            && Math.abs(testPointDefBotRelative.getY()) <= 0.45) {
          fieldPoints.add(testPointField);
          break;
        }
      }
    }

    Translation3d[] fieldPoints3d = new Translation3d[fieldPoints.size()];
    for (int i = 0; i < fieldPoints.size(); i++) {
      fieldPoints3d[i] =
          new Translation3d(
              fieldPoints.get(i).getX(), fieldPoints.get(i).getY(), robotToLidar.getZ());
    }

    inputs.fieldPoints = fieldPoints3d;
  }

  @Override
  public String getName() {
    return deviceName;
  }
}
