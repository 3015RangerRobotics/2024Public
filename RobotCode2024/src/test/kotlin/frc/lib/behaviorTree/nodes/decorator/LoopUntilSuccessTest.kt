package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class LoopUntilSuccessTest {
  @Test
  fun loopUntilSuccess() {
    val node = LoopUntilSuccess(SucceedAfter3Runs(""), "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  @Test
  fun handlesNull() {
    val node = LoopUntilSuccess(null, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  @Test
  fun testRunning() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = LoopUntilSuccess(testNode, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
  }

  internal class SucceedAfter3Runs(uuid: String) : LeafNode(uuid) {
    private var runCount = 0

    override fun handleInitialize(blackboard: Blackboard) {}

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      runCount++

      return if (runCount < 3) {
        ExecutionStatus.Failure
      } else {
        ExecutionStatus.Success
      }
    }

    override fun handleAbort(blackboard: Blackboard) {}
  }
}
