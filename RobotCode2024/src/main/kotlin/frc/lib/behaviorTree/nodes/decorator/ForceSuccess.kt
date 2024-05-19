package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class ForceSuccess(child: BehaviorTreeNode?, uuid: String) : DecoratorNode(child, uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val childStatus = child?.execute(blackboard)

    return if (childStatus == ExecutionStatus.Running) {
      ExecutionStatus.Running
    } else {
      ExecutionStatus.Success
    }
  }
}
