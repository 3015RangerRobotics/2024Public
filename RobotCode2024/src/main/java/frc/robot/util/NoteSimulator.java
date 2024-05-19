package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;

public class NoteSimulator {
  private static Pose3d currentFieldPose = new Pose3d();
  private static Translation3d fieldVelocity = new Translation3d();
  private static boolean inShooter = false;
  private static List<Translation3d> noteTrajectory = new ArrayList<>();

  private static final double AIR_DENSITY = 1.225;
  private static final double DRAG_COEFFICIENT = 0.45;
  private static final double CROSSECTION_AREA = 0.11;
  private static final double MASS = 0.235;

  public static void attachToShooter() {
    inShooter = true;
    noteTrajectory.clear();
  }

  public static boolean isAttached() {
    return inShooter;
  }

  public static List<Translation3d> getNoteTrajectory() {
    return noteTrajectory;
  }

  public static void launch(double velocity) {
    if (!inShooter) {
      return;
    }

    Pose3d armProximalPose = Robot.getArmProximalPose();
    Pose3d armDistalPose = Robot.getArmDistalPose(armProximalPose);
    Pose3d shooterPose = Robot.getShooterPose(armDistalPose, true);

    currentFieldPose = getFieldPose(shooterPose);
    inShooter = false;

    fieldVelocity = new Translation3d(velocity, currentFieldPose.getRotation());

    ChassisSpeeds robotVel = RobotContainer.swerve.getCurrentSpeeds();
    ChassisSpeeds fieldRel =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            robotVel, RobotContainer.swerve.getPose().getRotation());

    fieldVelocity =
        fieldVelocity.plus(
            new Translation3d(fieldRel.vxMetersPerSecond, fieldRel.vyMetersPerSecond, 0.0));
  }

  public static Pose3d getFieldPose(Pose3d shooterPose) {
    if (inShooter) {
      Pose3d robotRelative = shooterPose.transformBy(Constants.Shooter.shooterToNote);
      return new Pose3d(RobotContainer.swerve.getPose())
          .transformBy(
              new Transform3d(robotRelative.getTranslation(), robotRelative.getRotation()));
    }

    return currentFieldPose;
  }

  public static void update() {
    if (inShooter) {
      return;
    }

    double dt = 0.02;

    Translation3d posDelta = fieldVelocity.times(dt);

    currentFieldPose =
        new Pose3d(
            currentFieldPose.getTranslation().plus(posDelta), currentFieldPose.getRotation());

    if (currentFieldPose.getX() <= -0.25
        || currentFieldPose.getX() >= Constants.fieldSize.getX() + 0.25
        || currentFieldPose.getY() <= -0.25
        || currentFieldPose.getY() >= Constants.fieldSize.getY() + 0.25
        || currentFieldPose.getZ() <= 0.0) {
      fieldVelocity = new Translation3d();
    } else {
      fieldVelocity = fieldVelocity.minus(new Translation3d(0.0, 0.0, 9.81 * dt));
      double norm = fieldVelocity.getNorm();

      double fDrag = 0.5 * AIR_DENSITY * Math.pow(norm, 2) * DRAG_COEFFICIENT * CROSSECTION_AREA;
      double deltaV = (MASS * fDrag) * dt;

      double t = (norm - deltaV) / norm;
      fieldVelocity = fieldVelocity.times(t);
      noteTrajectory.add(currentFieldPose.getTranslation());
    }
  }
}
