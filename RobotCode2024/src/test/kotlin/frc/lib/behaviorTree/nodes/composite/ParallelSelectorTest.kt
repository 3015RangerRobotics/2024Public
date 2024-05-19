package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import frc.lib.behaviorTree.nodes.decorator.ForceFailure
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class ParallelSelectorTest {
  @Test
  fun parallelSelector() {
    val node1 = BehaviorTreeNodeTest.TestNode("")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf<BehaviorTreeNode>(node1, node2)
    val selector = ParallelSelector(nodes, "")
    val blackboard = Blackboard()

    selector.initialize(blackboard)
    Assertions.assertTrue(node1.initializeCalled)
    Assertions.assertTrue(node2.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, selector.execute(blackboard))
    Assertions.assertTrue(node1.executeCalled)
    Assertions.assertTrue(node2.executeCalled)

    Assertions.assertEquals(ExecutionStatus.Success, selector.execute(blackboard))
  }

  @Test
  fun abort() {
    val node1 = BehaviorTreeNodeTest.TestNode("")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf<BehaviorTreeNode>(node1, node2)
    val selector = ParallelSelector(nodes, "")
    val blackboard = Blackboard()

    selector.initialize(blackboard)
    selector.execute(blackboard)
    selector.abort(blackboard)

    Assertions.assertTrue(node1.abortCalled)
    Assertions.assertTrue(node2.abortCalled)
  }

  @Test
  fun handlesFailure() {
    val node1: BehaviorTreeNode = ForceFailure(BehaviorTreeNodeTest.TestNode(""), "")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf(node1, node2)
    val selector = ParallelSelector(nodes, "")
    val blackboard = Blackboard()

    selector.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Running, selector.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Success, selector.execute(blackboard))
  }

  @Test
  fun emptyChildren() {
    val sequence = ParallelSelector(emptyList(), "")
    val blackboard = Blackboard()

    sequence.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, sequence.execute(blackboard))
  }
}
