package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class CloseThree extends AutoCommand {

  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;

  public CloseThree() {
    path1 = PathPlannerPath.fromChoreoTrajectory("CloseThree.1");
    path2 = PathPlannerPath.fromChoreoTrajectory("CloseThree.2");
    path3 = PathPlannerPath.fromChoreoTrajectory("CloseThree.3");

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      addCommands(AutoBuildingBlocks.resetOdom(path1));
    }

    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoBuildingBlocks.shootPreload(0.3),
                AutoBuilder.followPath(path1).deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(path2).deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(path3).deadlineWith(AutoBuildingBlocks.intakeThenAim()),
                AutoBuildingBlocks.intakeAimAndShoot()),
            RobotContainer.shooter.setTargetSurfaceVelocityCommand(
                Constants.Shooter.shooterWheelSpeed)));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(path1.getPathPoses(), path2.getPathPoses(), path3.getPathPoses())
        .flatMap(Collection::stream)
        .collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return path1
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
