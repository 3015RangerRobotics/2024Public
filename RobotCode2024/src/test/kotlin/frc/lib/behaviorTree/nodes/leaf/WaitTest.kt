package frc.lib.behaviorTree.nodes.leaf

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.SimHooks
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.ResourceLock

class WaitTest {
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
  fun testWaitNode() {
    SimHooks.pauseTiming()

    val node = Wait(3.0, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))

    // Just hit abort for the coverage :)
    node.abort(blackboard)
  }
}
