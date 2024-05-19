package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class LoopUntilFailure(child: BehaviorTreeNode?, uuid: String) : DecoratorNode(child, uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val childStatus = child?.execute(blackboard) ?: ExecutionStatus.Failure

    return when (childStatus) {
      ExecutionStatus.Running -> ExecutionStatus.Running
      ExecutionStatus.Failure -> ExecutionStatus.Success
      ExecutionStatus.Success -> {
        child?.initialize(blackboard)
        ExecutionStatus.Running
      }
    }
  }
}
