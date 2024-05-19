package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

abstract class ParallelCompositeNode(
    children: List<BehaviorTreeNode>,
    private val endCondition: ExecutionStatus,
    uuid: String
) : BehaviorTreeNode(uuid) {
  private val statusMap: MutableMap<BehaviorTreeNode, ExecutionStatus?> = HashMap()

  init {
    statusMap.putAll(children.associateWith { null })
  }

  override fun handleInitialize(blackboard: Blackboard) {
    statusMap.keys.forEach { node ->
      statusMap[node] = ExecutionStatus.Running
      node.initialize(blackboard)
    }
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (statusMap.isEmpty()) {
      return ExecutionStatus.Success
    }

    for ((node, status) in statusMap) {
      if (status == ExecutionStatus.Running) {
        val newStatus = node.execute(blackboard)
        statusMap[node] = newStatus

        if (newStatus == endCondition) {
          abortRunning(blackboard)
          return endCondition
        }
      }
    }

    return if (statusMap.containsValue(ExecutionStatus.Running)) {
      ExecutionStatus.Running
    } else {
      endCondition.invert()
    }
  }

  private fun abortRunning(blackboard: Blackboard) {
    statusMap.entries
        .filter { it.value == ExecutionStatus.Running }
        .forEach { it.key.abort(blackboard) }
  }

  override fun handleAbort(blackboard: Blackboard) {
    abortRunning(blackboard)
  }
}
