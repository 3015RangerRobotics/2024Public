package frc.lib.behaviorTree.nodes

import frc.lib.behaviorTree.BehaviorTreeDebug
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

abstract class BehaviorTreeNode(private val uuid: String) {
  fun initialize(blackboard: Blackboard) {
    handleInitialize(blackboard)
    BehaviorTreeDebug.setActive(uuid)
  }

  protected abstract fun handleInitialize(blackboard: Blackboard)

  fun execute(blackboard: Blackboard): ExecutionStatus {
    val status = handleExecute(blackboard)

    if (status != ExecutionStatus.Running) {
      BehaviorTreeDebug.setInactive(uuid)
    }

    return status
  }

  protected abstract fun handleExecute(blackboard: Blackboard): ExecutionStatus

  fun abort(blackboard: Blackboard) {
    handleAbort(blackboard)
    BehaviorTreeDebug.setInactive(uuid)
  }

  protected abstract fun handleAbort(blackboard: Blackboard)
}
