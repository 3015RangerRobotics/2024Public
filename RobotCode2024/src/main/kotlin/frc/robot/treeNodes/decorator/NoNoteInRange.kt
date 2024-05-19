package frc.robot.treeNodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.decorator.ConditionDecorator
import frc.robot.RobotContainer

class NoNoteInRange(child: BehaviorTreeNode, uuid: String) : ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    val robotPose = RobotContainer.swerve.pose
    return true

    //    for (note in RobotContainer.objectDetection.filteredNoteDetections) {
    //      if (note
    //          .getFieldPos(Robot.currentZEDPose)
    //          .toTranslation2d()
    //          .getDistance(robotPose.translation) <= 2.5) {
    //        return false
    //      }
    //    }
    //
    //    return true
  }
}
