package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class LoopTest {
  @Test
  fun loop3Times() {
    val counter = CountRuns("")
    val loop = Loop(counter, 3, "")
    val blackboard = Blackboard()

    loop.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Success, loop.execute(blackboard))
    Assertions.assertEquals(3, counter.runs)
  }

  @Test
  fun loop5Times() {
    val counter = CountRuns("")
    val loop = Loop(counter, 5, "")
    val blackboard = Blackboard()

    loop.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Success, loop.execute(blackboard))
    Assertions.assertEquals(5, counter.runs)
  }

  @Test
  fun handlesNull() {
    val loop = Loop(null, 10, "")
    val blackboard = Blackboard()

    loop.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Success, loop.execute(blackboard))
  }

  @Test
  fun testRunning() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val node = Loop(testNode, 2, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
  }

  internal class CountRuns(uuid: String) : LeafNode(uuid) {
    var runs: Int = 0

    override fun handleInitialize(blackboard: Blackboard) {}

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      runs++
      return ExecutionStatus.Success
    }

    override fun handleAbort(blackboard: Blackboard) {}
  }
}
