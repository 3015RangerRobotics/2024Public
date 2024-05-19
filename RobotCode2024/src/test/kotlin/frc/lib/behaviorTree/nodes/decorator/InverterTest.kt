package frc.lib.behaviorTree.nodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import org.junit.jupiter.api.Assertions
import org.junit.jupiter.api.Test

class InverterTest {
  @Test
  fun running() {
    val node = Inverter(StatusNode(ExecutionStatus.Running, ""), "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Running, node.execute(blackboard))
  }

  @Test
  fun success() {
    val node = Inverter(StatusNode(ExecutionStatus.Success, ""), "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Failure, node.execute(blackboard))
  }

  @Test
  fun failure() {
    val node = Inverter(StatusNode(ExecutionStatus.Failure, ""), "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  @Test
  fun handlesNull() {
    val node = Inverter(null, "")
    val blackboard = Blackboard()

    node.initialize(blackboard)
    Assertions.assertEquals(ExecutionStatus.Success, node.execute(blackboard))
  }

  internal class StatusNode(private val status: ExecutionStatus, uuid: String) : LeafNode(uuid) {
    override fun handleInitialize(blackboard: Blackboard) {}

    override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
      return status
    }

    override fun handleAbort(blackboard: Blackboard) {}
  }
}