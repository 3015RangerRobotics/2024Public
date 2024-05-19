package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class SwerveAutoAlignPose extends Command {
  private final Pose2d redPose;
  private final Pose2d bluePose;

  private final ProfiledPIDController xController;
  private final ProfiledPIDController yController;
  private final ProfiledPIDController rotationController;

  private Pose2d targetPose;
  private final Timer timer = new Timer();

  public SwerveAutoAlignPose(Pose2d redPose, Pose2d bluePose) {
    this.redPose = redPose;
    this.bluePose = bluePose;

    this.xController =
        new ProfiledPIDController(
            Constants.Swerve.pathFollowingConfig.translationConstants.kP,
            Constants.Swerve.pathFollowingConfig.translationConstants.kI,
            Constants.Swerve.pathFollowingConfig.translationConstants.kD,
            new TrapezoidProfile.Constraints(2.0, 2.0));
    this.yController =
        new ProfiledPIDController(
            Constants.Swerve.pathFollowingConfig.translationConstants.kP,
            Constants.Swerve.pathFollowingConfig.translationConstants.kI,
            Constants.Swerve.pathFollowingConfig.translationConstants.kD,
            new TrapezoidProfile.Constraints(2.0, 2.0));
    this.rotationController =
        new ProfiledPIDController(
            Constants.Swerve.aimAtGoalConstants.kP,
            Constants.Swerve.aimAtGoalConstants.kI,
            Constants.Swerve.aimAtGoalConstants.kD,
            new TrapezoidProfile.Constraints(
                Units.degreesToRadians(180), Units.degreesToRadians(180)));
    this.rotationController.setIZone(Constants.Swerve.aimAtGoalConstants.iZone);
    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(RobotContainer.swerve);
  }

  @Override
  public void initialize() {
    if (Robot.isRedAlliance()) {
      targetPose = redPose;
    } else {
      targetPose = bluePose;
    }

    Pose2d currentPose = RobotContainer.swerve.getPose();
    ChassisSpeeds currentSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.swerve.getCurrentSpeeds(), currentPose.getRotation());

    xController.reset(currentPose.getX(), currentSpeeds.vxMetersPerSecond);
    yController.reset(currentPose.getY(), currentSpeeds.vyMetersPerSecond);
    rotationController.reset(
        currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
  }

  @Override
  public void execute() {
    Pose2d currentPose = RobotContainer.swerve.getPose();

    double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
    double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());
    double rotFeedback =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    double xFF = xController.getSetpoint().velocity;
    double yFF = yController.getSetpoint().velocity;
    double rotFF = rotationController.getSetpoint().velocity;

    double xVel = xFF + xFeedback;
    double yVel = yFF + yFeedback;
    double rotVel = rotFF + rotFeedback;

    if (Math.abs(currentPose.getX() - targetPose.getX()) < 0.025) {
      xVel = 0;
    }
    if (Math.abs(currentPose.getY() - targetPose.getY()) < 0.025) {
      yVel = 0;
    }
    if (Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15) {
      rotVel = 0;
    }

    RobotContainer.swerve.driveFieldRelative(new ChassisSpeeds(xVel, yVel, rotVel));
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = RobotContainer.swerve.getPose();
    return Math.abs(currentPose.getX() - targetPose.getX()) < 0.025
        && Math.abs(currentPose.getY() - targetPose.getY()) < 0.025
        && Math.abs(currentPose.getRotation().minus(targetPose.getRotation()).getDegrees()) < 15;
  }
}
