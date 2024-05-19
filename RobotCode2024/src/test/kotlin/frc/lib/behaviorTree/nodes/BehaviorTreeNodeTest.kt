package frc.lib.behaviorTree.nodes

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class BehaviorTreeNodeTest {
  @Test
  fun testNode() {
    val node = TestNode("")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertTrue(node.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    node.abort(blackboard)
    Assertions.assertTrue(node.abortCalled)
  }

  class TestNode(uuid: String) : BehaviorTreeNode(uuid) {
    var initializeCalled: Boolean = false
    var executeCalled: Boolean = false
    var abortCalled: Boolean = false

    override fun handleInitialize(blackboard: Blackboard) {
      initializeCalled = true
    }

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      return if (executeCalled) {
        ExecutionStatus.Success
      } else {
        executeCalled = true
        ExecutionStatus.Running
      }
    }

    override fun handleAbort(blackboard: Blackboard) {
      abortCalled = true
    }
  }
}
