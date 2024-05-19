package frc.robot.treeNodes.leaf

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.wpilibj2.command.Command
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.Constants
import frc.robot.Robot

class GoToSpeakerPos(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    val scoringPos =
        if (Robot.isRedAlliance()) {
          Constants.speakerShootPoseRed
        } else {
          Constants.speakerShootPoseBlue
        }

    return AutoBuilder.pathfindToPose(scoringPos, Constants.pathfindingConstraints, 0.0, 0.0)
  }
}
