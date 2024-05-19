package frc.lib.behaviorTree.nodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class ClearVariableTest {
  @Test
  fun clearVariable() {
    val node = ClearVariable("testKey", "")
    val blackboard = Blackboard()
    blackboard.setVariable("testKey", "testVal")

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))

    Assertions.assertFalse(blackboard.isVariableSet("testKey"))
  }
}
