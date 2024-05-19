package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class DecoratorNodeTest {
  @Test
  fun decorator() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val decorator = TestDecorator(testNode, "")
    val blackboard = Blackboard()

    decorator.initialize(blackboard)
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertEquals(ExecutionStatus.Running, decorator.execute(blackboard))
    Assertions.assertTrue(testNode.executeCalled)

    decorator.abort(blackboard)
    Assertions.assertTrue(testNode.abortCalled)
  }

  internal class TestDecorator(child: BehaviorTreeNode, uuid: String) : DecoratorNode(child, uuid) {
    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      return child!!.execute(blackboard)
    }
  }
}
