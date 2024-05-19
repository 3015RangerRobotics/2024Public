package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.RobotContainer

class WaitUntilNearChute(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val pickupPose =
        if (Robot.isRedAlliance()) {
          Constants.chutePoseRedFar
        } else {
          Constants.chutePoseBlueFar
        }

    return if (RobotContainer.swerve.pose.translation.getDistance(pickupPose.translation) <= 1.5) {
      ExecutionStatus.Success
    } else {
      ExecutionStatus.Running
    }
  }
}
