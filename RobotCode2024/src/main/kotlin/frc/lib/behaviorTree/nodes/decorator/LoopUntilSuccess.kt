package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class LoopUntilSuccess(child: BehaviorTreeNode?, uuid: String) : DecoratorNode(child, uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val childStatus = child?.execute(blackboard) ?: ExecutionStatus.Success

    return when (childStatus) {
      ExecutionStatus.Running -> ExecutionStatus.Running
      ExecutionStatus.Success -> ExecutionStatus.Success
      ExecutionStatus.Failure -> {
        child?.initialize(blackboard)
        ExecutionStatus.Running
      }
    }
  }
}
