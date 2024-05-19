package frc.lib.behaviorTree

import edu.wpi.first.hal.HAL
import frc.lib.behaviorTree.nodes.BehaviorTreeNodeTest
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.BeforeEach
import org.junit.jupiter.api.Test

class BehaviorTreeCommandTest {
  @BeforeEach
  fun setup() {
    HAL.initialize(500, 0)
  }

  @Test
  fun treeCommand() {
    val testNode = BehaviorTreeNodeTest.TestNode("")
    val cmd = BehaviorTreeCommand(testNode)

    cmd.initialize()
    Assertions.assertTrue(testNode.initializeCalled)

    Assertions.assertFalse(cmd.isFinished)

    cmd.execute()
    Assertions.assertTrue(testNode.executeCalled)
    Assertions.assertFalse(cmd.isFinished)

    cmd.execute()
    Assertions.assertTrue(cmd.isFinished)

    cmd.end(false)
    Assertions.assertFalse(testNode.abortCalled)

    cmd.end(true)
    Assertions.assertTrue(testNode.abortCalled)
  }
}
