package frc.lib.behaviorTree.nodes.leaf

import edu.wpi.first.wpilibj.Timer
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus

class Wait(private val waitTime: Double, uuid: String) : LeafNode(uuid) {
  private val timer: Timer = Timer()

  override fun handleInitialize(blackboard: Blackboard) {
    timer.restart()
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return if (timer.hasElapsed(waitTime)) ExecutionStatus.Success else ExecutionStatus.Running
  }
}
