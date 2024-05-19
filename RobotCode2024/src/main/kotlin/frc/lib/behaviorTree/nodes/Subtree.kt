package frc.lib.behaviorTree.nodes

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

class Subtree(private val rootNode: BehaviorTreeNode?, uuid: String) : BehaviorTreeNode(uuid) {
  override fun handleInitialize(blackboard: Blackboard) {
    rootNode?.initialize(blackboard)
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return rootNode?.execute(blackboard) ?: ExecutionStatus.Success
  }

  override fun handleAbort(blackboard: Blackboard) {
    rootNode?.abort(blackboard)
  }
}
