package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.leaf.CommandRunner
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.util.NoteSimulator

class ChuteIntake(uuid: String) : CommandRunner(uuid) {
  override fun buildCommand(blackboard: Blackboard): Command {
    return RobotContainer.chuteIntakeCommand()
        .alongWith(
            Commands.waitSeconds(2.0)
                .andThen({
                  if (Robot.isSimulation()) {
                    NoteSimulator.attachToShooter()
                  }
                }))
  }

  override fun handleAbort(blackboard: Blackboard) {
    // Prevent aboring the intake command to make sure it finishes
  }
}
