package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Collections;
import java.util.List;

public class NoneAuto extends AutoCommand {
  public NoneAuto() {
    addCommands(Commands.none());
  }

  @Override
  public List<Pose2d> getAllPathPoses() {
    return Collections.emptyList();
  }

  @Override
  public Pose2d getStartingPose() {
    return new Pose2d();
  }
}
