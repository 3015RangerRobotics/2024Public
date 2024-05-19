package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.RobotContainer

class SetGoodToIntake(uuid: String) : LeafNode(uuid) {
  var lastButtonValue = false

  override fun handleInitialize(blackboard: Blackboard) {
    if (!blackboard.isVariableSet("Autopilot")) {
      lastButtonValue = RobotContainer.driver.leftBumper
      blackboard.setVariable("GoodToIntake", lastButtonValue)
    }
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (!blackboard.isVariableSet("Autopilot")) {
      val pressed = RobotContainer.driver.leftBumper
      if (pressed != lastButtonValue) {
        lastButtonValue = pressed
        blackboard.setVariable("GoodToIntake", lastButtonValue)
      }
    }

    return ExecutionStatus.Running
  }
}
