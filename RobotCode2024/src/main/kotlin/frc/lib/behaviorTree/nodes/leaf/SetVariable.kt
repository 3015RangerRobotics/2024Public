package frc.lib.behaviorTree.nodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

class SetVariable(private val key: String, private val value: Any?, uuid: String) : LeafNode(uuid) {
  override fun handleInitialize(blackboard: Blackboard) {
    if (value != null) {
      blackboard.setVariable(key, value)
    }
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return ExecutionStatus.Success
  }
}
