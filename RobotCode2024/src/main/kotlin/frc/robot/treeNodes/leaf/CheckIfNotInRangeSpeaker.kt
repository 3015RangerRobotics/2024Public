package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj.DriverStation
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.RobotContainer

class CheckIfNotInRangeSpeaker(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val targetPos =
        if (DriverStation.getAlliance()
            .filter { e: DriverStation.Alliance -> e == DriverStation.Alliance.Red }
            .isPresent) {
          Constants.speakerPosRed
        } else {
          Constants.speakerPosBlue
        }

    return if (RobotContainer.swerve.pose.translation.getDistance(targetPos.toTranslation2d()) <=
        6.0) {
      ExecutionStatus.Failure
    } else {
      ExecutionStatus.Success
    }
  }
}
