package frc.lib.behaviorTree

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class BehaviorTreeCommand(private val rootNode: BehaviorTreeNode?) : Command() {
  private val blackboard: Blackboard = Blackboard()
  private var finished: Boolean = false

  companion object {
    val treeSubsystem: Subsystem = TreeSubsystem()

    fun stopActiveTree(): Command = treeSubsystem.runOnce {}
  }

  init {
    BehaviorTreeDebug.pushActive()

    addRequirements(treeSubsystem)
  }

  override fun initialize() {
    finished = false

    blackboard.clear()
    rootNode?.initialize(blackboard)
  }

  override fun execute() {
    val status = rootNode?.execute(blackboard)

    BehaviorTreeDebug.pushActive()

    if (status != ExecutionStatus.Running) {
      finished = true
    }
  }

  override fun isFinished(): Boolean = finished

  override fun end(interrupted: Boolean) {
    if (interrupted) {
      rootNode?.abort(blackboard)
    }
  }

  class TreeSubsystem : SubsystemBase()
}
