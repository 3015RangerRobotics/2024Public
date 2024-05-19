package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.RobotContainer

class WaitUntilCanShoot(uuid: String) : LeafNode(uuid) {
  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val targetPos =
        if (DriverStation.getAlliance().filter { e: Alliance -> e == Alliance.Red }.isPresent) {
          Constants.speakerPosRed
        } else {
          Constants.speakerPosBlue
        }

    return if (RobotContainer.swerve.pose.translation.getDistance(targetPos.toTranslation2d()) <=
        8.0) {
      ExecutionStatus.Success
    } else {
      ExecutionStatus.Running
    }
  }
}
