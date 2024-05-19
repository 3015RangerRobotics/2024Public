package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.behaviorTree.BehaviorTreeCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led_strip.LEDStrip;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

public class ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool
    extends SequentialCommandGroup {
  public ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool() {
    addCommands(
        BehaviorTreeCommand.Companion.stopActiveTree(),
        setObstacles(),
        pathfindToClimbPos().deadlineWith(RobotContainer.armToRestPos()),
        Commands.either(
            Commands.sequence(
                preClimbAndAlign(true),
                RobotContainer.armToPreClimbPos(true).withTimeout(0.5),
                climberSafetyCheck(true),
                Commands.either(
                    climbUpTrap().andThen(climbTrapScore()),
                    climbUpNoTrap(),
                    () -> DriverStation.getMatchTime() < 0 || DriverStation.getMatchTime() > 3)),
            Commands.sequence(
                preClimbAndAlign(false),
                RobotContainer.armToPreClimbPos(false).withTimeout(0.5),
                climberSafetyCheck(false),
                climbUpNoTrap()),
            RobotContainer.uptake::hasGamePieceDebounced));
  }

  private Command climberSafetyCheck(boolean trap) {

    return RobotContainer.armToPreClimbPos(trap)
        .raceWith(
            RobotContainer.climber
                .safetyClimber()
                .deadlineWith(
                    Commands.waitUntil(
                            () -> RobotContainer.ledStrip.getState() == LEDStrip.State.SAD)
                        .andThen(Commands.waitSeconds(0.5), Commands.runOnce(this::cancel))));
  }

  private Command pathfindToClimbPos() {
    return Commands.defer(
        () -> {
          Pose2d robotPose = RobotContainer.swerve.getPose();
          Pose2d targetPose =
              switch (Robot.getTargetClimbPos()) {
                case Left -> Robot.isRedAlliance()
                    ? Constants.climbLeftPathfindingPoseRed
                    : Constants.climbLeftPathfindingPoseBlue;
                case Right -> Robot.isRedAlliance()
                    ? Constants.climbRightPathfindingPoseRed
                    : Constants.climbRightPathfindingPoseBlue;
                default -> Robot.isRedAlliance()
                    ? Constants.climbCenterPathfindingPoseRed
                    : Constants.climbCenterPathfindingPoseBlue;
              };

          if (robotPose.getTranslation().getDistance(targetPose.getTranslation()) <= 0.25) {
            return Commands.none();
          } else if (robotPose.getTranslation().getDistance(targetPose.getTranslation()) <= 1.0) {
            Rotation2d heading =
                targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle();

            PathPlannerPath path =
                new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(
                        new Pose2d(robotPose.getTranslation(), heading),
                        new Pose2d(targetPose.getTranslation(), heading)),
                    Constants.pathfindingConstraints,
                    new GoalEndState(0, targetPose.getRotation(), true));
            path.preventFlipping = true;

            return AutoBuilder.followPath(path);
          } else {
            return AutoBuilder.pathfindToPose(targetPose, Constants.pathfindingConstraints);
          }
        },
        Set.of(RobotContainer.swerve));
  }

  private Command preClimbAndAlign(boolean trap) {
    return Commands.deadline(
        Commands.waitSeconds(0.5)
            .andThen(
                Commands.runOnce(
                    () ->
                        Commands.runOnce(() -> RobotContainer.localization.dontCorrect = true)
                            .andThen(
                                Commands.waitSeconds(2.0),
                                Commands.runOnce(
                                    () -> RobotContainer.localization.dontCorrect = false))
                            .schedule()),
                Commands.defer(
                    () -> {
                      Pose2d robotPose = RobotContainer.swerve.getPose();
                      Pose2d targetPose =
                          switch (Robot.getTargetClimbPos()) {
                            case Left -> Robot.isRedAlliance()
                                ? Constants.climbLeftPoseRed
                                : Constants.climbLeftPoseBlue;
                            case Right -> Robot.isRedAlliance()
                                ? Constants.climbRightPoseRed
                                : Constants.climbRightPoseBlue;
                            default -> Robot.isRedAlliance()
                                ? Constants.climbCenterPoseRed
                                : Constants.climbCenterPoseBlue;
                          };
                      Rotation2d heading =
                          targetPose.getTranslation().minus(robotPose.getTranslation()).getAngle();

                      PathPlannerPath path =
                          new PathPlannerPath(
                              PathPlannerPath.bezierFromPoses(
                                  new Pose2d(robotPose.getTranslation(), heading),
                                  new Pose2d(targetPose.getTranslation(), heading)),
                              new PathConstraints(
                                  1.0,
                                  1.0,
                                  Units.degreesToRadians(360),
                                  Units.degreesToRadians(540)),
                              new GoalEndState(0, targetPose.getRotation(), true));
                      path.preventFlipping = true;

                      return AutoBuilder.followPath(path);
                    },
                    Set.of(RobotContainer.swerve))),
        RobotContainer.armToPreClimbPos(trap),
        RobotContainer.climber.climberExtend());
  }

  public static Command climbUpTrap() {
    return RobotContainer.climber
        .climberRetract()
        .alongWith(
            RobotContainer.shooter.setRPMCommand(-500),
            RobotContainer.uptake
                .setRPMCommand(-100, false)
                .withTimeout(0.5)
                .andThen(RobotContainer.uptake.stopUptakeCommand()),
            Commands.waitSeconds(1.0).andThen(RobotContainer.armToTrapPos()))
        .withTimeout(3.0);
  }

  private Command climbUpNoTrap() {
    return RobotContainer.climber
        .climberRetract()
        .alongWith(RobotContainer.armToPreClimbPos(false));
  }

  public static Command climbTrapScore() {
    return RobotContainer.climber
        .climberRetract()
        .alongWith(
            RobotContainer.uptake
                .setVoltageCommand(-6)
                .withTimeout(2.0)
                .andThen(
                    Commands.parallel(
                        RobotContainer.uptake.stopUptakeCommand(),
                        RobotContainer.shooterJoint.setTargetAngleProfiledCommand(
                            Constants.ShooterJoint.postclimbPositionAngle),
                        RobotContainer.armJoint.setTargetAngleCommand(
                            Constants.ArmJoint.trapPositionAngle.plus(Rotation2d.fromDegrees(40))),
                        Commands.waitSeconds(0.25)
                            .andThen(RobotContainer.armExtension.setTargetExtensionCommand(0.0)))));
  }

  private Command setObstacles() {
    return Commands.runOnce(
        () -> {
          List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();

          if (Robot.isRedAlliance()) {
            obstacles.add(Pair.of(new Translation2d(5.9, 5.4), new Translation2d(3.2, 2.5)));

            switch (Robot.getTargetClimbPos()) {
              case Left -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbCenterPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbCenterPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbRightPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbRightPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
              case Right -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbCenterPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbCenterPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbLeftPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbLeftPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
              default -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbLeftPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbLeftPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbRightPoseRed.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbRightPoseRed
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
            }
          } else {
            obstacles.add(
                Pair.of(
                    GeometryUtil.flipFieldPosition(new Translation2d(5.9, 5.4)),
                    GeometryUtil.flipFieldPosition(new Translation2d(3.2, 2.5))));

            switch (Robot.getTargetClimbPos()) {
              case Left -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbCenterPoseBlue
                            .getTranslation()
                            .plus(new Translation2d(1, 1)),
                        Constants.climbCenterPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbRightPoseBlue.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbRightPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
              case Right -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbCenterPoseBlue
                            .getTranslation()
                            .plus(new Translation2d(1, 1)),
                        Constants.climbCenterPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbLeftPoseBlue.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbLeftPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
              default -> {
                obstacles.add(
                    Pair.of(
                        Constants.climbLeftPoseBlue.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbLeftPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
                obstacles.add(
                    Pair.of(
                        Constants.climbRightPoseBlue.getTranslation().plus(new Translation2d(1, 1)),
                        Constants.climbRightPoseBlue
                            .getTranslation()
                            .minus(new Translation2d(1, 1))));
              }
            }
          }

          Pathfinding.setDynamicObstacles(
              obstacles, RobotContainer.swerve.getPose().getTranslation());
        });
  }
}
