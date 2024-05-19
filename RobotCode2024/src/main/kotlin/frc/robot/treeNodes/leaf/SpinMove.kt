package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.commands.SwerveSpinMove

class SpinMove(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    val goalPose = blackboard.getPose2d("PathfindingGoalPose")

    return if (goalPose != null) {
      SwerveSpinMove(goalPose.translation)
    } else {
      Commands.none()
    }
  }
}
