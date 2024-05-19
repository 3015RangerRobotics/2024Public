package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class CheckVar(
    child: BehaviorTreeNode?,
    private val varKey: String,
    private val value: Any?,
    uuid: String
) : ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    return blackboard.isVariableSet(varKey) && blackboard.getVariable(varKey) == value
  }
}
