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
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class SixNoteBranching extends AutoCommand {
  public enum Priority {
    P12,
    P21
  }

  private final PathPlannerPath startToFirst;
  private final PathPlannerPath firstToFarShoot;
  private final PathPlannerPath firstToSecond;
  private final PathPlannerPath farShootToSecond;
  private final PathPlannerPath secondToCloseShoot;
  private final PathPlannerPath closeShootToThree;
  private final PathPlannerPath threeToFour;
  private final PathPlannerPath fourToFive;
  private final PathPlannerPath fiveToEnd;

  public SixNoteBranching(Priority priority) {
    String first = "";
    String second = "";

    switch (priority) {
      case P12 -> {
        first = "1";
        second = "2";
      }
      case P21 -> {
        first = "2";
        second = "1";
      }
    }
    startToFirst = PathPlannerPath.fromChoreoTrajectory("SixStartTo" + first);
    firstToFarShoot = PathPlannerPath.fromChoreoTrajectory("Six" + first + "ToFarShoot");
    farShootToSecond = PathPlannerPath.fromChoreoTrajectory("SixFarShootTo" + second);
    secondToCloseShoot = PathPlannerPath.fromChoreoTrajectory("Six" + second + "ToClose");
    closeShootToThree = PathPlannerPath.fromChoreoTrajectory("SixCloseToThree.2");
    threeToFour = PathPlannerPath.fromChoreoTrajectory("SixCloseToThree.3");
    fourToFive = PathPlannerPath.fromChoreoTrajectory("SixCloseToThree.4");
    firstToSecond = PathPlannerPath.fromChoreoTrajectory("Six" + first + "To" + second);
    fiveToEnd = PathPlannerPath.fromChoreoTrajectory("SixFiveToEnd");

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      addCommands(AutoBuildingBlocks.resetOdom(startToFirst));
    }

    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoBuildingBlocks.shootPreload(0.3),
                AutoBuilder.followPath(startToFirst)
                    .deadlineWith(Commands.waitSeconds(1.0).andThen(AutoBuildingBlocks.intake())),
                branchFromFirst(),
                AutoBuilder.followPath(secondToCloseShoot)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                Commands.deadline(
                    RobotContainer.uptake.shootUntilNoRing(), RobotContainer.armAimLow()),
                AutoBuilder.followPath(closeShootToThree)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(threeToFour)
                    .deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(fourToFive).deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(fiveToEnd)),
            RobotContainer.shooter.setTargetSurfaceVelocityCommand(
                Constants.Shooter.shooterWheelSpeed)));
  }

  private Command branchFromFirst() {
    return branch(
        // We have a note
        Commands.sequence(
            AutoBuilder.followPath(firstToFarShoot)
                .deadlineWith(
                    AutoBuildingBlocks.intakeThenHardcodeShot(Constants.ShooterJoint.farAutoAngle)),
            Commands.deadline(
                RobotContainer.uptake.shootUntilNoRing(),
                AutoBuildingBlocks.aimAtAngle(Constants.ShooterJoint.farAutoAngle)),
            AutoBuilder.followPath(farShootToSecond)
                .deadlineWith(Commands.waitSeconds(0.5).andThen(AutoBuildingBlocks.intake()))),
        // Missed the note
        AutoBuilder.followPath(firstToSecond).deadlineWith(AutoBuildingBlocks.intake()));
  }

  private Command branch(Command hasNote, Command noNote) {
    return Commands.either(
        hasNote,
        noNote,
        () -> RobotContainer.uptake.hasGamePiece() || RobotContainer.intake.getCurrentDraw() > 30);
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            startToFirst.getPathPoses(),
            firstToFarShoot.getPathPoses(),
            farShootToSecond.getPathPoses(),
            secondToCloseShoot.getPathPoses(),
            closeShootToThree.getPathPoses(),
            threeToFour.getPathPoses(),
            fourToFive.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return startToFirst
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
