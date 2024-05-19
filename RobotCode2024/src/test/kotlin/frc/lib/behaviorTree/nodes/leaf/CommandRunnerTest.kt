package frc.lib.behaviorTree.nodes.leaf

import edu.wpi.first.hal.HAL
import edu.wpi.first.wpilibj.simulation.SimHooks
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.AfterEach
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.parallel.ResourceLock

class CommandRunnerTest {
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
  fun testCommandRunner() {
    SimHooks.pauseTiming()

    val node = TestNode("")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    CommandScheduler.getInstance().run()
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    CommandScheduler.getInstance().run()
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    CommandScheduler.getInstance().run()
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))

    Assertions.assertFalse(node.aborted)
  }

  @Test
  @ResourceLock("timing")
  fun testAbort() {
    SimHooks.pauseTiming()

    val node = TestNode("")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    CommandScheduler.getInstance().run()
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    SimHooks.stepTiming(2.0)
    CommandScheduler.getInstance().run()
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))

    node.abort(blackboard)

    Assertions.assertTrue(node.aborted)
  }

  internal class TestNode(uuid: String) : CommandRunner(uuid) {
    var aborted: Boolean = false

    override fun buildCommand(blackboard: Blackboard): Command {
      return Commands.waitSeconds(3.0).handleInterrupt { aborted = true }
    }
  }
}
