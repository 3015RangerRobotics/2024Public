package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.RobotContainer

class WaitUntilHasRing(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return if (RobotContainer.uptake.hasGamePiece()) {
      ExecutionStatus.Success
    } else {
      ExecutionStatus.Running
    }
  }
}
