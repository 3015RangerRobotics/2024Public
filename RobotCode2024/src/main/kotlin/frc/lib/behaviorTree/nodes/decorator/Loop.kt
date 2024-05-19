package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class Loop(child: BehaviorTreeNode?, private val repeatTimes: Int, uuid: String) :
    DecoratorNode(child, uuid) {
  private var count: Int = 0

  override fun handleInitialize(blackboard: Blackboard) {
    super.handleInitialize(blackboard)
    count = 0
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (child == null) {
      return ExecutionStatus.Success
    }

    val childStatus = child.execute(blackboard)

    if (childStatus == ExecutionStatus.Running) {
      return ExecutionStatus.Running
    }

    count++

    return if (count >= repeatTimes) {
      childStatus
    } else {
      child.initialize(blackboard)
      ExecutionStatus.Running
    }
  }
}
