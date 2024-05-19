package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.RobotContainer

class SetHasGamePiece(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    blackboard.setVariable("HasGamePiece", RobotContainer.uptake.hasGamePieceDebounced())
    return ExecutionStatus.Success
  }
}
