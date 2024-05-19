package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.Robot
import frc.robot.RobotContainer

class DriveAimAtSpeakerPass(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return RobotContainer.driveAimLaunchAngle().onlyWhile {
      Robot.getScoringMode() == Robot.ScoringMode.Pass &&
          RobotContainer.uptake.hasGamePieceDebounced()
    }
  }
}
