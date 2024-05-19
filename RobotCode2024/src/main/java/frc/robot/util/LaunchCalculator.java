package frc.robot.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LaunchCalculator {
  private static LaunchState currentLaunchState =
      new LaunchState(new Rotation3d(), Constants.Shooter.shooterExitSpeed, false);

  public static LaunchState getCurrentLaunchState() {
    return currentLaunchState;
  }

  public static void updateLaunch(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double nominalLaunchVelocity,
      ChassisSpeeds robotVelocityField,
      boolean rising) {
    currentLaunchState =
        calculateDynamicLaunch(
            targetPosField, shooterPosField, nominalLaunchVelocity, robotVelocityField, rising);
    //    currentLaunchState =
    //        calculateStaticLaunch(targetPosField, shooterPosField, nominalLaunchVelocity);
  }

  private static LaunchState calculateStaticLaunch(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double launchVelocity,
      boolean rising) {
    Rotation3d launchAngle =
        calculateStaticLaunchAngle(targetPosField, shooterPosField, launchVelocity, rising);

    if (launchAngle == null) {
      return new LaunchState(new Rotation3d(), launchVelocity, false);
    }

    return new LaunchState(launchAngle, launchVelocity, true);
  }

  private static double calculateStaticLaunchYaw(
      Translation3d targetPosField, Translation3d shooterPosField) {
    return targetPosField
        .minus(shooterPosField)
        .toTranslation2d()
        .getAngle()
        .minus(Rotation2d.fromDegrees(2))
        .getRadians();
  }

  private static double calculateStaticLaunchPitchNoAir(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double launchVelocity,
      boolean rising) {
    double h = shooterPosField.getZ() - targetPosField.getZ();
    double x = shooterPosField.toTranslation2d().getDistance(targetPosField.toTranslation2d());

    double thing =
        ((9.8 * Math.pow(x, 2) / Math.pow(launchVelocity, 2)) - h)
            / Math.sqrt(Math.pow(h, 2) + Math.pow(x, 2));
    if (rising) {
      double phi = Math.atan(h / x);
      return (Math.asin(thing) - phi) / 2.0;
    } else {
      double phi = Math.atan(x / h);
      return (Math.acos(thing) + phi) / 2.0;
    }
  }

  private static Rotation3d calculateStaticLaunchAngle(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double launchVelocity,
      boolean rising) {
    double pitch =
        calculateStaticLaunchPitchNoAir(targetPosField, shooterPosField, launchVelocity, rising);
    double yaw = calculateStaticLaunchYaw(targetPosField, shooterPosField);

    if (!Double.isFinite(pitch) || !Double.isFinite(yaw)) {
      return null;
    }

    return new Rotation3d(0.0, pitch, yaw);
  }

  private static Translation3d calculateDynamicLaunchVector(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double launchVelocity,
      ChassisSpeeds robotVelocityField,
      boolean rising) {
    Rotation3d staticLaunchAngle =
        calculateStaticLaunchAngle(targetPosField, shooterPosField, launchVelocity, rising);

    if (staticLaunchAngle == null) {
      return null;
    }

    Rotation3d test =
        staticLaunchAngle.rotateBy(new Pose3d(RobotContainer.swerve.getPose()).getRotation());
    Rotation3d test2 = new Rotation3d(test.getX(), test.getY(), staticLaunchAngle.getZ());
    Translation3d staticLaunchVec = new Translation3d(launchVelocity, test2);
    // z is wrong idk
    staticLaunchVec =
        new Translation3d(staticLaunchVec.getX(), staticLaunchVec.getY(), -staticLaunchVec.getZ());

    // cancel out the velocity of the shooter on the field
    Translation3d velocityTrans =
        new Translation3d(
            robotVelocityField.vxMetersPerSecond, robotVelocityField.vyMetersPerSecond, 0.0);
    return staticLaunchVec.minus(velocityTrans);
  }

  private static LaunchState calculateDynamicLaunch(
      Translation3d targetPosField,
      Translation3d shooterPosField,
      double launchVelocity,
      ChassisSpeeds robotVelocityField,
      boolean rising) {
    Translation3d launchVec =
        calculateDynamicLaunchVector(
            targetPosField, shooterPosField, launchVelocity, robotVelocityField, rising);

    if (launchVec == null) {
      return new LaunchState(new Rotation3d(), launchVelocity, false);
    }

    Rotation2d staticYaw =
        new Rotation2d(calculateStaticLaunchYaw(targetPosField, shooterPosField));
    Rotation2d yaw = new Rotation2d(launchVec.getX(), launchVec.getY());

    var rotated = launchVec.rotateBy(new Rotation3d(0.0, 0.0, yaw.unaryMinus().getRadians()));
    Rotation2d pitch =
        new Rotation2d(rotated.getX(), rotated.getZ()).plus(Constants.Shooter.exitAngleOffset);

    Rotation3d launchAngle = new Rotation3d(0.0, pitch.getRadians(), yaw.getRadians());

    double velocity = rotated.getNorm();

    return new LaunchState(launchAngle, velocity, true);
  }

  public record LaunchState(Rotation3d launchAngle, double launchVelocity, boolean valid) {}
}
