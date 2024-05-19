package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

public abstract class AutoCommand extends SequentialCommandGroup {
  public abstract List<Pose2d> getAllPathPoses();

  public abstract Pose2d getStartingPose();
}
