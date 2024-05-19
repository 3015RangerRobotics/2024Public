package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class ParallelSequence(children: List<BehaviorTreeNode>, uuid: String) :
    ParallelCompositeNode(children, ExecutionStatus.Failure, uuid)
