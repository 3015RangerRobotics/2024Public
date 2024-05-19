package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.RobotContainer
import frc.robot.subsystems.uptake.Uptake

class Shoot(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return RobotContainer.uptake
        .shootUntilNoRing()
        .alongWith(Commands.runOnce({ Uptake.hasGamePieceSimOverride = false }))
    //        .until { !RobotContainer.uptake.hasGamePieceDebounced() }
  }
}
