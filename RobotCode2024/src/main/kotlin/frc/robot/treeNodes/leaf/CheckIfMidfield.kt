package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.RobotContainer

class CheckIfMidfield(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val currentPose = RobotContainer.swerve.pose

    return if (currentPose.x in 6.25..10.29) {
      ExecutionStatus.Success
    } else {
      ExecutionStatus.Failure
    }
  }
}
