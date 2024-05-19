package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class ConditionDecoratorTest {
  @Test
  fun testTrue() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val condition = TrueCondition(testNode, "")
    val blackboard = Blackboard()

    condition.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, condition.execute(blackboard))
    Assertions.assertTrue(testNode.executeCalled)
  }

  @Test
  fun testFalse() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val condition = FalseCondition(testNode, "")
    val blackboard = Blackboard()

    condition.initialize(blackboard)
    Assertions.assertFalse(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Failure, condition.execute(blackboard))
    Assertions.assertFalse(testNode.executeCalled)
  }

  internal class TrueCondition(child: BehaviorTreeNode, uuid: String) :
      ConditionDecorator(child, uuid) {
    override fun condition(blackboard: Blackboard): Boolean {
      return true
    }
  }

  internal class FalseCondition(child: BehaviorTreeNode, uuid: String) :
      ConditionDecorator(child, uuid) {
    override fun condition(blackboard: Blackboard): Boolean {
      return false
    }
  }
}
