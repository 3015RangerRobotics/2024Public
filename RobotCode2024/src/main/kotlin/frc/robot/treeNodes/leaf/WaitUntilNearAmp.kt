package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.RobotContainer

class WaitUntilNearAmp(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val ampPos =
        if (Robot.isRedAlliance()) {
          Constants.targetAmpPoseRed.translation
        } else {
          Constants.targetAmpPoseBlue.translation
        }

    return if (RobotContainer.swerve.pose.translation.getDistance(ampPos) <= 3.0) {
      ExecutionStatus.Success
    } else {
      ExecutionStatus.Running
    }
  }
}
