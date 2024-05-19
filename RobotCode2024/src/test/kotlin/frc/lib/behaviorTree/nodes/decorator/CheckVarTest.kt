package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class CheckVarTest {
  @Test
  fun equalVar() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val check = CheckVar(testNode, "key", "val", "")
    val blackboard = Blackboard()

    blackboard.setVariable("key", "val")

    check.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, check.execute(blackboard))
  }

  @Test
  fun unequalVar() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val check = CheckVar(testNode, "key", "val", "")
    val blackboard = Blackboard()

    blackboard.setVariable("key", 15)

    check.initialize(blackboard)
    Assertions.assertFalse(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Failure, check.execute(blackboard))
  }
}
