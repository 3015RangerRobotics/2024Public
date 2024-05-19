package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class Inverter(child: BehaviorTreeNode?, uuid: String) : DecoratorNode(child, uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val childStatus = child?.execute(blackboard)

    return childStatus?.invert() ?: ExecutionStatus.Success
  }
}
