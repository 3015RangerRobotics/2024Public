package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SourceBranching extends AutoCommand {
  public enum Priority {
    P123,
    P132,
    P213,
    P231,
    P312,
    P321
  }

  private final PathPlannerPath startToFirst;
  private final PathPlannerPath firstToShoot;
  private final PathPlannerPath shootToSecond;
  private final PathPlannerPath secondToShoot;
  private final PathPlannerPath shootToThird;
  private final PathPlannerPath thirdToShoot;
  private final PathPlannerPath firstToSecond;
  private final PathPlannerPath secondToThird;
  private final PathPlannerPath shootToEnd;

  public SourceBranching(Priority priority) {
    String first = "";
    String second = "";
    String third = "";

    switch (priority) {
      case P123 -> {
        first = "1";
        second = "2";
        third = "3";
      }
      case P132 -> {
        first = "1";
        second = "3";
        third = "2";
      }
      case P213 -> {
        first = "2";
        second = "1";
        third = "3";
      }
      case P231 -> {
        first = "2";
        second = "3";
        third = "1";
      }
      case P312 -> {
        first = "3";
        second = "1";
        third = "2";
      }
      case P321 -> {
        first = "3";
        second = "2";
        third = "1";
      }
    }

    startToFirst = PathPlannerPath.fromChoreoTrajectory("SourceStartTo" + first);
    firstToShoot = PathPlannerPath.fromChoreoTrajectory("Source" + first + "ToShoot");
    shootToSecond = PathPlannerPath.fromChoreoTrajectory("SourceShootTo" + second);
    secondToShoot = PathPlannerPath.fromChoreoTrajectory("Source" + second + "ToShoot");
    shootToThird = PathPlannerPath.fromChoreoTrajectory("SourceShootTo" + third);
    thirdToShoot = PathPlannerPath.fromChoreoTrajectory("Source" + third + "ToShoot");
    firstToSecond = PathPlannerPath.fromChoreoTrajectory("Source" + first + "To" + second);
    secondToThird = PathPlannerPath.fromChoreoTrajectory("Source" + second + "To" + third);
    shootToEnd = PathPlannerPath.fromChoreoTrajectory("SourceShootToEnd");

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      addCommands(AutoBuildingBlocks.resetOdom(startToFirst));
    }

    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoBuildingBlocks.shootPreload(0.5)
                    .deadlineWith(RobotContainer.driveAimLaunchAngle()),
                AutoBuilder.followPath(startToFirst).deadlineWith(AutoBuildingBlocks.intake()),
                branchFromFirst()),
            RobotContainer.shooter.setTargetSurfaceVelocityCommand(
                Constants.Shooter.shooterWheelSpeed)));
  }

  private Command branchFromFirst() {
    return branch(
            // We have a note
            Commands.sequence(
                AutoBuilder.followPath(firstToShoot)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                shoot(),
                AutoBuilder.followPath(shootToSecond).deadlineWith(AutoBuildingBlocks.intake())),
            // Missed the note
            AutoBuilder.followPath(firstToSecond).deadlineWith(AutoBuildingBlocks.intake()))
        .andThen(branchFromSecond());
  }

  private Command branchFromSecond() {
    return branch(
            // We have a note
            Commands.sequence(
                AutoBuilder.followPath(secondToShoot)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                shoot(),
                AutoBuilder.followPath(shootToThird).deadlineWith(AutoBuildingBlocks.intake())),
            // Missed the note
            AutoBuilder.followPath(secondToThird).deadlineWith(AutoBuildingBlocks.intake()))
        .andThen(branchFromThird());
  }

  private Command branchFromThird() {
    return branch(
        // We have a note
        Commands.sequence(
            AutoBuilder.followPath(thirdToShoot).deadlineWith(AutoBuildingBlocks.intakeThenAim()),
            shoot(),
            AutoBuilder.followPath(shootToEnd).deadlineWith(AutoBuildingBlocks.intake())),
        // Missed the note (keep intaking in hopes we get it then shoot)
        Commands.sequence(
            AutoBuildingBlocks.intake(),
            Commands.sequence(
                AutoBuilder.followPath(thirdToShoot)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                shoot(),
                AutoBuilder.followPath(shootToEnd).deadlineWith(AutoBuildingBlocks.intake()))));
  }

  private Command branch(Command hasNote, Command noNote) {
    return Commands.either(
        hasNote,
        noNote,
        () -> RobotContainer.uptake.hasGamePiece() || RobotContainer.intake.getCurrentDraw() > 30);
  }

  private Command shoot() {
    return Commands.deadline(
        Commands.waitSeconds(0.1).andThen(RobotContainer.uptake.shootUntilNoRing()),
        RobotContainer.armAimLow(),
        RobotContainer.driveAimLaunchAngle());
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return startToFirst.getPathPoses();
  }

  @Override
  public Pose2d getStartingPose() {
    return startToFirst
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
