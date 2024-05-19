package frc.robot.commands.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmToPosition;
import frc.robot.subsystems.uptake.Uptake;
import frc.robot.util.NoteSimulator;

public class AutoBuildingBlocks {
  public static Command resetOdom(PathPlannerPath choreoPath) {
    return RobotContainer.swerve.runOnce(
        () -> {
          Pose2d pose =
              choreoPath
                  .getTrajectory(new ChassisSpeeds(), new Rotation2d())
                  .getInitialTargetHolonomicPose();
          if (Robot.isRedAlliance()) {
            pose = GeometryUtil.flipFieldPose(pose);
          }

          RobotContainer.swerve.resetOdom(pose);
        });
  }

  public static Command shootPreload(double time) {
    return Commands.sequence(
        Commands.runOnce(NoteSimulator::attachToShooter),
        Commands.deadline(
            Commands.sequence(
                Commands.waitSeconds(time),
                RobotContainer.uptake
                    .shootUntilNoRing()
                    .alongWith(Commands.runOnce(() -> Uptake.hasGamePieceSimOverride = false))),
            RobotContainer.armAimLow()));
  }

  public static Command shootPreload(double time, Rotation2d angle) {
    return Commands.sequence(
        Commands.runOnce(NoteSimulator::attachToShooter),
        Commands.deadline(
            Commands.sequence(
                Commands.waitSeconds(time),
                RobotContainer.uptake
                    .shootUntilNoRing()
                    .alongWith(Commands.runOnce(() -> Uptake.hasGamePieceSimOverride = false))),
            aimAtAngle(angle)));
  }

  public static Command intakeThenHardcodeShot(Rotation2d angle) {
    return Commands.sequence(
        RobotContainer.armToRestPos()
            .raceWith(
                Commands.waitSeconds(0.25)
                    .andThen(
                        RobotContainer.intakeUptakeCommand()
                            .repeatedly()
                            .until(RobotContainer.uptake::hasGamePieceDebounced))),
        aimAtAngle(angle)
            .alongWith(
                RobotContainer.intake.stopIntakeCommand(),
                RobotContainer.uptake.holdPositionCommand()));
  }

  public static Command aimAtAngle(Rotation2d angle) {
    return new ArmToPosition(
        Constants.ArmJoint.restPositionAngle, Constants.ArmExtension.shootPositionExtension, angle);
  }

  public static Command intakeAimAndShoot() {
    return Commands.either(
        Commands.either(
                Commands.none(),
                RobotContainer.intakeUptakeCommand()
                    .deadlineWith(RobotContainer.armToRestPos())
                    .withTimeout(1.0),
                RobotContainer.uptake::hasGamePiece)
            .andThen(
                Commands.deadline(
                    Commands.sequence(
                            Commands.waitSeconds(0.1)
                                .andThen(
                                    Commands.waitUntil(
                                        () ->
                                            Math.abs(
                                                    RobotContainer.shooterJoint
                                                        .getAngle()
                                                        .minus(
                                                            RobotContainer.shooterJoint
                                                                .getTargetAngle())
                                                        .getDegrees())
                                                <= 2.0)),
                            RobotContainer.uptake
                                .shootUntilNoRing()
                                .alongWith(
                                    Commands.runOnce(() -> Uptake.hasGamePieceSimOverride = false)))
                        .deadlineWith(RobotContainer.armAimLow()),
                    RobotContainer.driveAimLaunchAngle())),
        Commands.none(),
        () -> RobotContainer.uptake.hasGamePiece() || RobotContainer.intake.getCurrentDraw() > 30);
  }

  public static Command intakeThenAim() {
    return Commands.sequence(
        RobotContainer.armToRestPos()
            .raceWith(
                Commands.waitSeconds(0.25)
                    .andThen(
                        RobotContainer.intakeUptakeCommand()
                            .repeatedly()
                            .until(RobotContainer.uptake::hasGamePieceDebounced))),
        RobotContainer.armAimLow()
            .alongWith(
                RobotContainer.intake.stopIntakeCommand(),
                RobotContainer.uptake.stopUptakeCommand()));
  }

  public static Command intake() {
    return Commands.sequence(
        RobotContainer.armToRestPos()
            .raceWith(
                Commands.waitSeconds(0.25)
                    .andThen(
                        RobotContainer.intakeUptakeCommand()
                            .repeatedly()
                            .until(RobotContainer.uptake::hasGamePieceDebounced))),
        Commands.parallel(
            RobotContainer.intake.stopIntakeCommand(), RobotContainer.uptake.stopUptakeCommand()));
  }
}
