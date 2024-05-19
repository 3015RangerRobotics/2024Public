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

public class AmpBread extends AutoCommand {
  private final PathPlannerPath path1;
  private final PathPlannerPath path2;
  private final PathPlannerPath path3;
  private final PathPlannerPath path4;
  private final PathPlannerPath path5;
  private final PathPlannerPath path6;
  private final PathPlannerPath path7;

  private final PathPlannerPath path8;

  public AmpBread() {
    path1 = PathPlannerPath.fromChoreoTrajectory("AmpBread.1");
    path2 = PathPlannerPath.fromChoreoTrajectory("AmpBread.2");
    path3 = PathPlannerPath.fromChoreoTrajectory("AmpBread.3");
    path4 = PathPlannerPath.fromChoreoTrajectory("AmpBread.4");
    path5 = PathPlannerPath.fromChoreoTrajectory("AmpBread.5");
    path6 = PathPlannerPath.fromChoreoTrajectory("AmpBread.6");
    path7 = PathPlannerPath.fromChoreoTrajectory("AmpBread.7");
    path8 = PathPlannerPath.fromChoreoTrajectory("AmpBread.8");

    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      addCommands(AutoBuildingBlocks.resetOdom(path1));
    }

    addCommands(
        Commands.deadline(
            Commands.sequence(
                RobotContainer.driveAimLaunchAngle().raceWith(AutoBuildingBlocks.shootPreload(0.6)),
                AutoBuilder.followPath(path1).deadlineWith(AutoBuildingBlocks.intake()),
                AutoBuildingBlocks.intakeAimAndShoot(),
                AutoBuilder.followPath(path2).deadlineWith(AutoBuildingBlocks.intake()),
                AutoBuilder.followPath(path3)
                    .deadlineWith(
                        AutoBuildingBlocks.intakeThenHardcodeShot(
                            Constants.ShooterJoint.farAutoAngle)),
                Commands.deadline(
                    RobotContainer.uptake.shootUntilNoRing(),
                    AutoBuildingBlocks.aimAtAngle(Constants.ShooterJoint.farAutoAngle)),
                AutoBuilder.followPath(path4).deadlineWith(AutoBuildingBlocks.intake()),
                AutoBuilder.followPath(path5)
                    .deadlineWith(
                        AutoBuildingBlocks.intakeThenHardcodeShot(
                            Constants.ShooterJoint.farAutoAngle)),
                Commands.deadline(
                    RobotContainer.uptake.shootUntilNoRing(),
                    AutoBuildingBlocks.aimAtAngle(Constants.ShooterJoint.farAutoAngle)),
                AutoBuilder.followPath(path6).deadlineWith(AutoBuildingBlocks.intake()),
                AutoBuilder.followPath(path7)
                    .deadlineWith(
                        AutoBuildingBlocks.intakeThenHardcodeShot(
                            Constants.ShooterJoint.farAutoAngle)),
                Commands.deadline(
                    RobotContainer.uptake.shootUntilNoRing(),
                    AutoBuildingBlocks.aimAtAngle(Constants.ShooterJoint.farAutoAngle)),
                AutoBuilder.followPath(path8).deadlineWith(AutoBuildingBlocks.intakeThenAim())),
            RobotContainer.shooter.setTargetSurfaceVelocityCommand(
                Constants.Shooter.shooterWheelSpeed)));
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Stream.of(
            path1.getPathPoses(),
            path2.getPathPoses(),
            path3.getPathPoses(),
            path4.getPathPoses(),
            path5.getPathPoses(),
            path6.getPathPoses(),
            path7.getPathPoses(),
            path8.getPathPoses())
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
