package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class IsVarUnsetTest {
  @Test
  fun varSet() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = IsVarUnset(testNode, "testKey", "")
    val blackboard = Blackboard()

    blackboard.setVariable("testKey", true)

    node.initialize(blackboard)
    Assertions.assertFalse(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Failure, node.execute(blackboard))
  }

  @Test
  fun varUnset() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = IsVarUnset(testNode, "testKey", "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
  }

  @Test
  fun handlesNull() {
    val node = IsVarUnset(null, "testKey", "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))

    blackboard.setVariable("testKey", true)
    Assertions.assertEquals(ExecutionStatus.Failure, node.execute(blackboard))
  }
}
