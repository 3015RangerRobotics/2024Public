package frc.lib.behaviorTree.nodes.decorator

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.SimHooks
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.ResourceLock

class TimeLimitTest {
  @BeforeEach
  fun setup() {
    HAL.initialize(500, 0)
  }

  @AfterEach
  fun cleanup() {
    SimHooks.resumeTiming()
  }

  @Test
  @ResourceLock("timing")
  fun timeLimit() {
    SimHooks.pauseTiming()

    val node = TimeLimit(RunningNode(""), 3.0, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    Assertions.assertEquals(ExecutionStatus.Failure, node.execute(blackboard))
  }

  @Test
  fun handlesNull() {
    val node = TimeLimit(null, 3.0, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  internal class RunningNode(uuid: String) : LeafNode(uuid) {
    override fun handleInitialize(blackboard: Blackboard) {}

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      return ExecutionStatus.Running
    }

    override fun handleAbort(blackboard: Blackboard) {}
  }
}
