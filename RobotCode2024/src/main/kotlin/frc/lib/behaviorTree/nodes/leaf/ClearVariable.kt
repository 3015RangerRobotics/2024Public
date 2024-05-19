package frc.lib.behaviorTree.nodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

class ClearVariable(private val key: String, uuid: String) : LeafNode(uuid) {
  override fun handleInitialize(blackboard: Blackboard) {
    blackboard.clearVariable(key)
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return ExecutionStatus.Success
  }
}
