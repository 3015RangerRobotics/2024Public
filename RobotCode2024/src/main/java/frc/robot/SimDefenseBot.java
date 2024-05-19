package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.input.controllers.XboxControllerWrapper;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class SimDefenseBot extends SubsystemBase {
  private final XboxControllerWrapper controller;
  private final DefenseBotInputsAutoLogged inputs = new DefenseBotInputsAutoLogged();

  private final SlewRateLimiter xStickLimiter =
      new SlewRateLimiter(Constants.Swerve.teleStickRateLimit);
  private final SlewRateLimiter yStickLimiter =
      new SlewRateLimiter(Constants.Swerve.teleStickRateLimit);
  private final SlewRateLimiter rotStickLimiter =
      new SlewRateLimiter(Constants.Swerve.teleStickRateLimit);

  public SimDefenseBot(int controllerPort) {
    this.controller = new XboxControllerWrapper(controllerPort, 0.3);

    xStickLimiter.reset(-controller.getLeftY());
    yStickLimiter.reset(-controller.getLeftX());
    rotStickLimiter.reset(-controller.getRightX());
  }

  @Override
  public void periodic() {
    double x = xStickLimiter.calculate(-controller.getLeftY()) * 4.5;
    double y = yStickLimiter.calculate(-controller.getLeftX()) * 4.5;
    double rot = rotStickLimiter.calculate(-controller.getRightX()) * 360;

    Translation2d deltaPos = new Translation2d(x, y).times(0.02);
    Rotation2d deltaRot = Rotation2d.fromDegrees(rot).times(0.02);

    inputs.pose =
        new Pose2d(
            inputs.pose.getTranslation().plus(deltaPos), inputs.pose.getRotation().plus(deltaRot));

    Logger.processInputs("DefenseBot", inputs);

    double[] logPose =
        new double[] {
          inputs.pose.getX(), inputs.pose.getY(), inputs.pose.getRotation().getRadians()
        };

    Logger.recordOutput("DefenseBot/Pose", logPose);
  }

  public Pose2d getPose() {
    return inputs.pose;
  }

  @AutoLog
  public static class DefenseBotInputs {
    Pose2d pose = new Pose2d();
  }
}
