package frc.lib.behaviorTree.nodes.composite

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import frc.lib.behaviorTree.nodes.decorator.ForceFailure
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class ParallelSequenceTest {
  @Test
  fun parallelSequence() {
    val node1 = BehaviorTreeNodeTest.TestNode("")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf<BehaviorTreeNode>(node1, node2)
    val sequence = ParallelSequence(nodes, "")
    val blackboard = Blackboard()

    sequence.initialize(blackboard)
    Assertions.assertTrue(node1.initializeCalled)
    Assertions.assertTrue(node2.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, sequence.execute(blackboard))
    Assertions.assertTrue(node1.executeCalled)
    Assertions.assertTrue(node2.executeCalled)

    Assertions.assertEquals(ExecutionStatus.Success, sequence.execute(blackboard))
  }

  @Test
  fun abort() {
    val node1 = BehaviorTreeNodeTest.TestNode("")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf<BehaviorTreeNode>(node1, node2)
    val sequence = ParallelSequence(nodes, "")
    val blackboard = Blackboard()

    sequence.initialize(blackboard)
    sequence.execute(blackboard)
    sequence.abort(blackboard)

    Assertions.assertTrue(node1.abortCalled)
    Assertions.assertTrue(node2.abortCalled)
  }

  @Test
  fun handlesFailure() {
    val node1: BehaviorTreeNode = ForceFailure(BehaviorTreeNodeTest.TestNode(""), "")
    val node2 = BehaviorTreeNodeTest.TestNode("")
    val nodes = listOf(node1, node2)
    val sequence = ParallelSequence(nodes, "")
    val blackboard = Blackboard()

    sequence.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Running, sequence.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Failure, sequence.execute(blackboard))
  }

  @Test
  fun emptyChildren() {
    val sequence = ParallelSequence(emptyList(), "")
    val blackboard = Blackboard()

    sequence.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, sequence.execute(blackboard))
  }
}
