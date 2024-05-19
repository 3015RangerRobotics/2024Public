package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class ParallelSelector(children: List<BehaviorTreeNode>, uuid: String) :
    ParallelCompositeNode(children, ExecutionStatus.Success, uuid)
