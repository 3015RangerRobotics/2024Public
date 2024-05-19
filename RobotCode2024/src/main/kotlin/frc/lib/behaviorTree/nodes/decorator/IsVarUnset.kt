package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class IsVarUnset(child: BehaviorTreeNode?, private val varKey: String, uuid: String) :
    ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    return !blackboard.isVariableSet(varKey)
  }
}
