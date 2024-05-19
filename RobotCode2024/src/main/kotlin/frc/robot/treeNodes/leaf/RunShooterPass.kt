package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.RobotContainer

class RunShooterPass(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return RobotContainer.shooter.setLaunchVelocityCommand()
  }
}
