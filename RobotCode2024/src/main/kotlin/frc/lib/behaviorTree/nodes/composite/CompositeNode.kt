package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

abstract class CompositeNode(
    private val children: List<BehaviorTreeNode>,
    private val endCondition: ExecutionStatus,
    uuid: String
) : BehaviorTreeNode(uuid) {
  private var iterator: ListIterator<BehaviorTreeNode> = children.listIterator()
  private var currentNode: BehaviorTreeNode? = null

  override fun handleInitialize(blackboard: Blackboard) {
    iterator = children.listIterator()
    pickNextNode(blackboard)
  }

  private fun pickNextNode(blackboard: Blackboard) {
    currentNode = if (iterator.hasNext()) iterator.next() else null
    currentNode?.initialize(blackboard)
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (children.isEmpty()) {
      return ExecutionStatus.Success
    }

    val currentNodeStatus = currentNode?.execute(blackboard) ?: ExecutionStatus.Failure
    if (currentNodeStatus == ExecutionStatus.Running || currentNodeStatus == endCondition) {
      return currentNodeStatus
    }

    return if (iterator.hasNext()) {
      pickNextNode(blackboard)
      handleExecute(blackboard)
    } else {
      currentNodeStatus
    }
  }

  override fun handleAbort(blackboard: Blackboard) {
    currentNode?.abort(blackboard)
  }
}
