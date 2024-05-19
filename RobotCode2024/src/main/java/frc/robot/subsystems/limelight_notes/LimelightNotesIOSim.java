package frc.robot.subsystems.limelight_notes;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.List;

public class LimelightNotesIOSim implements LimelightNotesIO {
  private final List<Translation3d> notePositions =
      List.of(
          new Translation3d(2.9, 7.0, 0.025),
          new Translation3d(2.9, 5.55, 0.025),
          new Translation3d(2.9, 4.1, 0.025),
          new Translation3d(8.29, 7.44, 0.025),
          new Translation3d(8.29, 5.78, 0.025),
          new Translation3d(8.29, 4.09, 0.025),
          new Translation3d(8.29, 2.44, 0.025),
          new Translation3d(8.29, 0.77, 0.025),
          new Translation3d(13.67, 4.10, 0.025),
          new Translation3d(13.67, 5.55, 0.025),
          new Translation3d(13.67, 7.0, 0.025),
          new Translation3d(15.5, 0.75, 0.025),
          new Translation3d(1.04, 0.75, 0.025));

  @Override
  public void updateInputs(LimelightNotesInputs inputs) {
    Pose3d robotPose = new Pose3d(RobotContainer.swerve.getPose());
    Pose3d llPose =
        robotPose.transformBy(
            new Transform3d(
                Robot.currentZEDPoseRobot.getTranslation(),
                Robot.currentZEDPoseRobot.getRotation()));

    Translation3d target = null;
    for (Translation3d pos : notePositions) {
      Rotation2d angleToObj =
          pos.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);

      if (Math.abs(diff.getDegrees()) <= (63.3 / 2.0)) {
        double distance = llPose.getTranslation().getDistance(pos);
        if (distance <= 6.0 && target == null) {
          target = pos;
        } else if (target != null && distance < llPose.getTranslation().getDistance(target)) {
          target = pos;
        }
      }
    }

    if (target != null) {
      inputs.hasTarget = true;
      inputs.timestamp = Timer.getFPGATimestamp();

      Rotation2d angleToObj =
          target.toTranslation2d().minus(llPose.getTranslation().toTranslation2d()).getAngle();
      Rotation2d diff = llPose.getRotation().toRotation2d().minus(angleToObj);
      inputs.tx = diff.getDegrees();

      double h = llPose.getTranslation().getDistance(target);
      double o = llPose.getZ() - target.getZ();
      inputs.ty = -Units.radiansToDegrees(Math.asin(o / h) - llPose.getRotation().getY());
    } else {
      inputs.hasTarget = false;
    }

    inputs.fps = 30.0;
    inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
  }
}
