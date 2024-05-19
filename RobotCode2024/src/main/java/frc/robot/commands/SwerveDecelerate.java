package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SwerveDecelerate extends Command {
  private final SlewRateLimiter limiter;
  private final double endVel;

  private Rotation2d directionOfTravel;
  private boolean isFinished;

  public SwerveDecelerate(double maxAccel, double endVel) {
    this.limiter = new SlewRateLimiter(maxAccel);
    this.endVel = endVel;

    addRequirements(RobotContainer.swerve);
  }

  @Override
  public void initialize() {
    ChassisSpeeds currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.swerve.getCurrentSpeeds(),
            RobotContainer.swerve.getPose().getRotation());

    directionOfTravel =
        new Rotation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    double currentVel =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

    limiter.reset(currentVel);
    isFinished = false;
  }

  @Override
  public void execute() {
    double nextVel = limiter.calculate(endVel);

    isFinished = Math.abs(nextVel - endVel) < 0.1;

    Translation2d speeds = new Translation2d(nextVel, directionOfTravel);

    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(speeds.getX(), speeds.getY(), 0.0));
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
