package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class LoopUntilFailureTest {
  @Test
  fun loopUntilFailure() {
    val node = LoopUntilFailure(FailAfter3Runs(""), "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  @Test
  fun handlesNull() {
    val node = LoopUntilFailure(null, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  @Test
  fun testRunning() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = LoopUntilFailure(testNode, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
  }

  internal class FailAfter3Runs(uuid: String) : LeafNode(uuid) {
    private var runCount = 0

    override fun handleInitialize(blackboard: Blackboard) {}

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      runCount++

      return if (runCount < 3) {
        ExecutionStatus.Success
      } else {
        ExecutionStatus.Failure
      }
    }

    override fun handleAbort(blackboard: Blackboard) {}
  }
}
