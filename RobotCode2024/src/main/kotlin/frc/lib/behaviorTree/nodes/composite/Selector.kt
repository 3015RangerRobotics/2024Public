package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class Selector(children: List<BehaviorTreeNode>, uuid: String) :
    CompositeNode(children, ExecutionStatus.Success, uuid)
