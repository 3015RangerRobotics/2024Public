package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.RobotContainer

class ArmToShootingPosDynamic(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return Commands.either(
            RobotContainer.armAimHigh().onlyWhile { shouldAimHigh(blackboard) },
            RobotContainer.armAimLow().onlyWhile { !shouldAimHigh(blackboard) }) {
              shouldAimHigh(blackboard)
            }
        .repeatedly()
  }

  private fun shouldAimHigh(blackboard: Blackboard): Boolean {
    return RobotContainer.driver.yButton && blackboard.getBoolean("GoodToScore") == true
  }
}
