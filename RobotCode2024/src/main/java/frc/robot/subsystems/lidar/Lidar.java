package frc.robot.subsystems.lidar;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Lidar extends AdvancedSubsystem {
  private final LidarIO[] io;
  private final LidarInputsAutoLogged[] inputs;

  private final List<LidarDetection> currentObstacles = new ArrayList<>();
  private List<LidarDetection> currentRobotObstacles = new ArrayList<>();

  public Lidar(LidarIO... io) {
    this.io = io;
    this.inputs = new LidarInputsAutoLogged[this.io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new LidarInputsAutoLogged();
    }
  }

  @Override
  public void periodic() {
    double startTime = Logger.getRealTimestamp();

    for (int i = 0; i < io.length; i++) {
      if (!Logger.hasReplaySource()) {
        io[i].updateInputs(inputs[i]);
      }
      Logger.processInputs("Lidar/" + io[i].getName(), inputs[i]);
    }

    List<Pose3d> logPoses = new ArrayList<>();
    currentObstacles.clear();
    for (var input : inputs) {
      List<LidarDetection> lidarDetections = detectClusters(input.fieldPoints);
      currentObstacles.addAll(lidarDetections);
    }

    currentRobotObstacles = getEstimatedRobotDetections();

    for (LidarDetection robot : currentRobotObstacles) {
      Translation3d min = robot.boundingBoxMin();
      Translation3d max = robot.boundingBoxMax();

      Translation3d p2 = new Translation3d(min.getX(), max.getY(), min.getZ());
      Translation3d p3 = new Translation3d(max.getX(), min.getY(), min.getZ());

      logPoses.add(new Pose3d(min, new Rotation3d(0.0, Units.degreesToRadians(90), 0.0)));
      logPoses.add(new Pose3d(p2, new Rotation3d(0.0, Units.degreesToRadians(90), 0.0)));
      logPoses.add(new Pose3d(p3, new Rotation3d(0.0, Units.degreesToRadians(90), 0.0)));
      logPoses.add(new Pose3d(max, new Rotation3d(0.0, Units.degreesToRadians(90), 0.0)));
      logPoses.add(
          new Pose3d(
              robot.boundingBoxCenter(), new Rotation3d(0.0, Units.degreesToRadians(90), 0.0)));
    }

    Logger.recordOutput("Lidar/FieldPoses", logPoses.toArray(new Pose3d[0]));

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Lidar/PeriodicMS", runtimeMS);
  }

  public List<LidarDetection> getCurrentDetections() {
    return currentObstacles;
  }

  public List<LidarDetection> getCurrentRobotDetections() {
    return currentRobotObstacles;
  }

  private List<LidarDetection> getEstimatedRobotDetections() {
    List<LidarDetection> detections = getCurrentDetections();
    List<LidarDetection> expectedRobots = new ArrayList<>();

    for (LidarDetection detection : detections) {
      if (detection.radius() >= 0.1 && detection.radius() <= 0.75) {
        if (expectedRobots.isEmpty()) {
          expectedRobots.add(detection);
        } else {
          boolean added = false;

          for (int i = 0; i < expectedRobots.size(); i++) {
            if (expectedRobots.get(i).boundingBoxCenter().getDistance(detection.boundingBoxCenter())
                <= 1.3) {
              Translation3d min1 = expectedRobots.get(i).boundingBoxMin();
              Translation3d min2 = detection.boundingBoxMin();
              Translation3d combinedMin =
                  new Translation3d(
                      Math.min(min1.getX(), min2.getX()),
                      Math.min(min1.getY(), min2.getY()),
                      Math.min(min1.getZ(), min2.getZ()));

              Translation3d max1 = expectedRobots.get(i).boundingBoxMax();
              Translation3d max2 = detection.boundingBoxMax();
              Translation3d combinedMax =
                  new Translation3d(
                      Math.max(max1.getX(), max2.getX()),
                      Math.max(max1.getY(), max2.getY()),
                      Math.max(max1.getZ(), max2.getZ()));

              Translation3d combinedCenter = combinedMin.interpolate(combinedMax, 0.5);
              double combinedRadius = combinedMin.getDistance(combinedCenter);

              expectedRobots.set(
                  i, new LidarDetection(combinedCenter, combinedMin, combinedMax, combinedRadius));
              added = true;
              break;
            }
          }

          if (!added) {
            expectedRobots.add(detection);
          }
        }
      }
    }

    // Expand non-square bounding boxes to encompass the whole robot
    Pose2d robotPose2d = RobotContainer.swerve.getPose();
    for (int i = 0; i < expectedRobots.size(); i++) {
      LidarDetection det = expectedRobots.get(i);

      double sizeX = det.boundingBoxMax().getX() - det.boundingBoxMin().getX();
      double sizeY = det.boundingBoxMax().getY() - det.boundingBoxMin().getY();
      double aspectRatio = sizeX / sizeY;

      if (aspectRatio > 0.75 && aspectRatio < 1.5) {
        continue;
      }

      Translation3d newMin;
      Translation3d newMax;

      if (sizeY < sizeX) {
        // Expand on Y axis
        if (robotPose2d.getTranslation().getY() < det.boundingBoxCenter().getY()) {
          // Expand +Y
          newMin = det.boundingBoxMin();
          newMax =
              new Translation3d(
                  det.boundingBoxMax().getX(), newMin.getY() + sizeX, det.boundingBoxMax().getZ());
        } else {
          // Expand -Y
          newMax = det.boundingBoxMax();
          newMin =
              new Translation3d(
                  det.boundingBoxMin().getX(), newMax.getY() - sizeX, det.boundingBoxMin().getZ());
        }
      } else {
        // Expand on X axis
        if (robotPose2d.getTranslation().getX() < det.boundingBoxCenter().getX()) {
          // Expand +X
          newMin = det.boundingBoxMin();
          newMax =
              new Translation3d(
                  newMin.getX() + sizeY, det.boundingBoxMax().getY(), det.boundingBoxMax().getZ());
        } else {
          // Expand -X
          newMax = det.boundingBoxMax();
          newMin =
              new Translation3d(
                  newMax.getX() - sizeY, det.boundingBoxMin().getY(), det.boundingBoxMin().getZ());
        }
      }

      Translation3d newCenter = newMin.interpolate(newMax, 0.5);
      double newRadius = newCenter.getDistance(newMin);

      expectedRobots.set(i, new LidarDetection(newCenter, newMin, newMax, newRadius));
    }

    return expectedRobots;
  }

  List<LidarDetection> detectClusters(Translation3d[] points) {
    List<LidarDetection> clusters = new ArrayList<>();

    if (points.length < Constants.Lidar.clusterMinCount) {
      return clusters;
    }

    List<List<Translation3d>> clusterPoints = new ArrayList<>();
    clusterPoints.add(new ArrayList<>(List.of(points[0])));

    for (Translation3d point : points) {
      if (point.getX() <= Constants.Lidar.fieldWallThreshold
          || point.getX() >= Constants.fieldSize.getX() - Constants.Lidar.fieldWallThreshold
          || point.getY() <= Constants.Lidar.fieldWallThreshold
          || point.getY() >= Constants.fieldSize.getY() - Constants.Lidar.fieldWallThreshold) {
        // Filter out points near/outside the field wall
        continue;
      }

      var last = clusterPoints.get(clusterPoints.size() - 1);
      if (point.getDistance(last.get(last.size() - 1)) < Constants.Lidar.clusterDistanceThreshold) {
        last.add(point);
      } else {
        clusterPoints.add(new ArrayList<>(List.of(point)));
      }
    }

    for (List<Translation3d> cluster : clusterPoints) {
      int numPoints = cluster.size();

      if (numPoints >= Constants.Lidar.clusterMinCount) {
        double minX = cluster.get(0).getX();
        double minY = cluster.get(0).getY();
        double minZ = cluster.get(0).getZ();

        double maxX = cluster.get(0).getX();
        double maxY = cluster.get(0).getY();
        double maxZ = cluster.get(0).getZ();

        for (int i = 1; i < cluster.size(); i++) {
          Translation3d p = cluster.get(i);

          if (p.getX() < minX) {
            minX = p.getX();
          } else if (p.getX() > maxX) {
            maxX = p.getX();
          }

          if (p.getY() < minY) {
            minY = p.getY();
          } else if (p.getY() > maxY) {
            maxY = p.getY();
          }

          if (p.getZ() < minZ) {
            minZ = p.getZ();
          } else if (p.getZ() > maxZ) {
            maxZ = p.getZ();
          }
        }

        Translation3d min = new Translation3d(minX, minY, minZ);
        Translation3d max = new Translation3d(maxX, maxY, maxZ);
        Translation3d center = min.interpolate(max, 0.5);
        double radius = min.getDistance(center);

        clusters.add(new LidarDetection(center, min, max, radius));
      }
    }

    return clusters;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }
}
