package frc.lib.behaviorTree.nodes

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class SubtreeTest {
  @Test
  fun handlesNull() {
    val node = Subtree(null, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
    node.abort(blackboard)
  }

  @Test
  fun runsNode() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = Subtree(testNode, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
    Assertions.assertTrue(testNode.executeCalled)

    node.abort(blackboard)
    Assertions.assertTrue(testNode.abortCalled)
  }
}
