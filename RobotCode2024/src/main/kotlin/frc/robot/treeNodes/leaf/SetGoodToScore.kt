package frc.robot.treeNodes.leaf

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.RobotContainer

class SetGoodToScore(uuid: String) : LeafNode(uuid) {
  var lastButtonValue = false

  override fun handleInitialize(blackboard: Blackboard) {
    if (!blackboard.isVariableSet("Autopilot")) {
      lastButtonValue = RobotContainer.driver.rightBumper
      blackboard.setVariable("GoodToScore", lastButtonValue)
    }
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    if (!blackboard.isVariableSet("Autopilot")) {
      val pressed = RobotContainer.driver.rightBumper
      if (pressed != lastButtonValue) {
        lastButtonValue = pressed
        blackboard.setVariable("GoodToScore", lastButtonValue)
      }
    }

    return ExecutionStatus.Running
  }
}
