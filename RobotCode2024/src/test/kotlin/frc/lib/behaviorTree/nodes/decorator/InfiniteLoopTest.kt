package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class InfiniteLoopTest {
  @Test
  fun loopInf() {
    val counter = CountRuns("")
    val loop = InfiniteLoop(counter, "")
    val blackboard = Blackboard()

    loop.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))

    Assertions.assertEquals(5, counter.runs)
  }

  @Test
  fun handlesNull() {
    val loop = InfiniteLoop(null, "")
    val blackboard = Blackboard()

    loop.initialize(blackboard)

    Assertions.assertEquals(ExecutionStatus.Running, loop.execute(blackboard))
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
