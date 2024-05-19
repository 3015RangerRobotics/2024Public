package frc.robot.commands;

import com.pathplanner.lib.util.ChassisSpeedsRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.lidar.LidarDetection;
import java.util.ArrayList;
import java.util.List;

public class SwerveSpinMove extends Command {
  private final Translation2d goalPathfindingPos;
  private final ChassisSpeedsRateLimiter limiter;

  private boolean isFinished;

  public SwerveSpinMove(Translation2d goalPathfindingPos) {
    this.goalPathfindingPos = goalPathfindingPos;
    this.limiter = new ChassisSpeedsRateLimiter(7.0, Units.degreesToRadians(900));

    addRequirements(RobotContainer.swerve);
  }

  @Override
  public void initialize() {
    isFinished = false;

    ChassisSpeeds fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.swerve.getCurrentSpeeds(),
            RobotContainer.swerve.getPose().getRotation());
    limiter.reset(fieldRelative);
  }

  @Override
  public void execute() {
    Pose2d robotPose2d = RobotContainer.swerve.getPose();

    //    List<LidarDetection> predictedRobots = RobotContainer.lidar.getCurrentRobotDetections();
    List<LidarDetection> predictedRobots = new ArrayList<>();

    if (predictedRobots.isEmpty()) {
      isFinished = true;
      return;
    }

    Translation2d obsPos = predictedRobots.get(0).boundingBoxCenter().toTranslation2d();
    double dist = obsPos.getDistance(robotPose2d.getTranslation());
    for (int i = 1; i < predictedRobots.size(); i++) {
      double d =
          predictedRobots
              .get(i)
              .boundingBoxCenter()
              .toTranslation2d()
              .getDistance(robotPose2d.getTranslation());
      if (d < dist) {
        obsPos = predictedRobots.get(i).boundingBoxCenter().toTranslation2d();
        dist = d;
      }
    }

    Rotation2d angleToObs = obsPos.minus(robotPose2d.getTranslation()).getAngle();

    Rotation2d angleToGoal = goalPathfindingPos.minus(robotPose2d.getTranslation()).getAngle();

    Rotation2d angleToObsGoalRel = angleToObs.minus(angleToGoal);

    if (Math.abs(angleToObsGoalRel.getDegrees()) >= 100) {
      Translation2d speedHack = new Translation2d(3.5, angleToGoal);
      RobotContainer.swerve.driveFieldRelative(
          new ChassisSpeeds(speedHack.getX(), speedHack.getY(), 0.0));

      if (obsPos.getDistance(robotPose2d.getTranslation()) >= 1.0) {
        isFinished = true;
      }
      return;
    }

    Rotation2d moveDirectionGoalRel = Rotation2d.fromDegrees(90.0);
    if (angleToObsGoalRel.getRadians() > 0) {
      moveDirectionGoalRel = moveDirectionGoalRel.times(-1);
    }
    moveDirectionGoalRel = moveDirectionGoalRel.plus(angleToObsGoalRel);

    Rotation2d moveDirectionFieldRel = moveDirectionGoalRel.plus(angleToGoal);

    Translation2d speedHack = new Translation2d(3.5, moveDirectionFieldRel);

    double rotSpeed = Units.degreesToRadians(450);
    if (angleToObsGoalRel.getRadians() < 0) {
      rotSpeed *= -1;
    }

    ChassisSpeeds output = new ChassisSpeeds(speedHack.getX(), speedHack.getY(), rotSpeed);

    RobotContainer.swerve.driveFieldRelative(output);
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
