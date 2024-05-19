package frc.lib.behaviorTree.nodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

abstract class LeafNode(uuid: String) : BehaviorTreeNode(uuid) {
  override fun handleInitialize(blackboard: Blackboard) {}

  override fun handleAbort(blackboard: Blackboard) {}
}
