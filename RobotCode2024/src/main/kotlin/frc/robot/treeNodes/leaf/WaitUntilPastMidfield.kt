package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Robot
import frc.robot.RobotContainer

class WaitUntilPastMidfield(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (Robot.getScoringMode() == Robot.ScoringMode.Pass) {
      return ExecutionStatus.Success
    }

    return if (Robot.isRedAlliance()) {
      if (RobotContainer.swerve.pose.x >= 8.25) {
        ExecutionStatus.Success
      } else {
        ExecutionStatus.Running
      }
    } else {
      if (RobotContainer.swerve.pose.x <= 8.25) {
        ExecutionStatus.Success
      } else {
        ExecutionStatus.Running
      }
    }
  }
}
