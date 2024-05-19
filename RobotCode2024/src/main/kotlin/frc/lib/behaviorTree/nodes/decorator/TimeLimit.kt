package frc.lib.behaviorTree.nodes.decorator

import edu.wpi.first.wpilibj.Timer
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.BehaviorTreeNode

class TimeLimit(child: BehaviorTreeNode?, private val timeLimit: Double, uuid: String) :
    DecoratorNode(child, uuid) {
  private val timer: Timer = Timer()

  override fun handleInitialize(blackboard: Blackboard) {
    child?.initialize(blackboard)

    timer.restart()
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    return if (timer.hasElapsed(timeLimit)) {
      child?.abort(blackboard)
      ExecutionStatus.Failure
    } else {
      child?.execute(blackboard) ?: ExecutionStatus.Success
    }
  }
}
