package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.Logger;

public class Trap extends AutoCommand {

  private final PathPlannerPath path1 = PathPlannerPath.fromChoreoTrajectory("trap");

  public Trap() {
    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      addCommands(AutoBuildingBlocks.resetOdom(path1));
    }

    addCommands(
        Commands.deadline(
                Commands.sequence(
                    AutoBuildingBlocks.shootPreload(0.3),
                    AutoBuilder.followPath(path1).deadlineWith(AutoBuildingBlocks.intake()),
                    Commands.waitSeconds(0.25).deadlineWith(AutoBuildingBlocks.intake())),
                RobotContainer.shooter.setLaunchVelocityCommand())
            .andThen(new ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool())
            .withTimeout(14),
        RobotContainer.climberUnclimb());
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(path1.getPathPoses()).flatMap(Collection::stream).collect(Collectors.toList());
  }

  @Override
  public Pose2d getStartingPose() {
    return path1
        .getTrajectory(new ChassisSpeeds(), new Rotation2d())
        .getInitialTargetHolonomicPose();
  }
}
