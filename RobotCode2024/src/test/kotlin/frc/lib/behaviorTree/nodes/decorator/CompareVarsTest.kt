package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class CompareVarsTest {
  @Test
  fun equalVars() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val compare = CompareVars(testNode, "keyA", "keyB", "")
    val blackboard = Blackboard()

    blackboard.setVariable("keyA", 15)
    blackboard.setVariable("keyB", 15)

    compare.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, compare.execute(blackboard))
  }

  @Test
  fun unequalVars() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val compare = CompareVars(testNode, "keyA", "keyB", "")
    val blackboard = Blackboard()

    blackboard.setVariable("keyA", 15)
    blackboard.setVariable("keyB", 10)

    compare.initialize(blackboard)
    Assertions.assertFalse(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Failure, compare.execute(blackboard))
  }

  @Test
  fun unsetVars() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val compare = CompareVars(testNode, "keyA", "keyB", "")
    val blackboard = Blackboard()

    compare.initialize(blackboard)
    Assertions.assertFalse(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Failure, compare.execute(blackboard))
  }
}
