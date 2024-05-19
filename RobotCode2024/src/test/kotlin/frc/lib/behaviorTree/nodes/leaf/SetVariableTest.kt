package frc.lib.behaviorTree.nodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class SetVariableTest {
  @Test
  fun setVariable() {
    val node = SetVariable("testKey", "testVal", "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
    Assertions.assertEquals("testVal", blackboard.getVariable("testKey"))
  }
}
