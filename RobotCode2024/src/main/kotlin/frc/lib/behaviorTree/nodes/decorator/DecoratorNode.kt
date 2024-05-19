package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

abstract class DecoratorNode(protected val child: BehaviorTreeNode?, uuid: String) :
    BehaviorTreeNode(uuid) {
  override fun handleInitialize(blackboard: Blackboard) {
    child?.initialize(blackboard)
  }

  override fun handleAbort(blackboard: Blackboard) {
    child?.abort(blackboard)
  }
}
