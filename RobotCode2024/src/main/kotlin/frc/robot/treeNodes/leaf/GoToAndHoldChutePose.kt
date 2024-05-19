package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.RobotContainer

class GoToAndHoldChutePose(uuid: String) : CommandRunner(uuid) {
  val cmd = RobotContainer.pathfindAndAlignChute()

  override fun buildCommand(blackboard: Blackboard): Command {
    return cmd
  }
}
