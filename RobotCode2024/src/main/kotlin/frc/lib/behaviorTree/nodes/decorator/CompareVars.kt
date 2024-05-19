package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class CompareVars(
    child: BehaviorTreeNode?,
    private val varKeyA: String,
    private val varKeyB: String,
    uuid: String
) : ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    return if (blackboard.isVariableSet(varKeyA) && blackboard.isVariableSet(varKeyB)) {
      blackboard.getVariable(varKeyA) == blackboard.getVariable(varKeyB)
    } else {
      false
    }
  }
}
