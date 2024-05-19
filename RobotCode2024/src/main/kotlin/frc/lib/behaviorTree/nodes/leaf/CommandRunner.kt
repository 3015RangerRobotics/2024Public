package frc.lib.behaviorTree.nodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

abstract class CommandRunner(uuid: String) : LeafNode(uuid) {
  protected lateinit var command: Command
  private var firstLoop: Boolean = false

  protected abstract fun buildCommand(blackboard: Blackboard): Command

  override fun handleInitialize(blackboard: Blackboard) {
    command = buildCommand(blackboard)

    command.schedule()
    firstLoop = true
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return if (!command.isScheduled && !firstLoop) {
      ExecutionStatus.Success
    } else {
      firstLoop = false
      ExecutionStatus.Running
    }
  }

  override fun handleAbort(blackboard: Blackboard) {
    if (command.isScheduled) {
      command.cancel()
    }
  }
}
