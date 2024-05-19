package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

abstract class ConditionDecorator(child: BehaviorTreeNode?, uuid: String) :
    DecoratorNode(child, uuid) {
  private var childRunning: Boolean = false

  override fun handleInitialize(blackboard: Blackboard) {
    childRunning = false

    if (child != null && condition(blackboard)) {
      child.initialize(blackboard)
      childRunning = true
    }
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (condition(blackboard)) {
      if (!childRunning && child != null) {
        child.initialize(blackboard)
        childRunning = true
      }
      return child?.execute(blackboard) ?: ExecutionStatus.Success
    }

    // Condition is now false
    if (childRunning) {
      child?.abort(blackboard)
    }
    return ExecutionStatus.Failure
  }

  protected abstract fun condition(blackboard: Blackboard): Boolean
}
