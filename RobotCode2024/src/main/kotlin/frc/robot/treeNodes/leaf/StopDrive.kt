package frc.robot.treeNodes.leaf

import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.RobotContainer

class StopDrive(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return Commands.run({ RobotContainer.swerve.driveRobotRelative(ChassisSpeeds()) })
  }
}
