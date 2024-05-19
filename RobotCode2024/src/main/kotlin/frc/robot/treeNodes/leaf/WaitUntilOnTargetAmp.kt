package frc.robot.treeNodes.leaf

import edu.wpi.first.wpilibj.Timer
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.RobotContainer
import kotlin.math.absoluteValue
import kotlin.math.hypot

class WaitUntilOnTargetAmp(uuid: String) : LeafNode(uuid) {
  val timer = Timer()

  override fun handleInitialize(blackboard: Blackboard) {
    timer.restart()
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    val currentPose = RobotContainer.swerve.pose

    val currentSpeeds = RobotContainer.swerve.currentSpeeds
    val currentVel = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)

    val targetPose =
        if (Robot.isRedAlliance()) {
          Constants.targetAmpPoseRed
        } else {
          Constants.targetAmpPoseBlue
        }
    val positionError = currentPose.translation.getDistance(targetPose.translation)
    val rotationError = targetPose.rotation.minus(currentPose.rotation)

    val wristError = RobotContainer.shooterJoint.angle - RobotContainer.shooterJoint.targetAngle
    val armJointError = RobotContainer.armJoint.angle - RobotContainer.armJoint.targetAngle
    val armExtensionError =
        RobotContainer.armExtension.extensionMeters -
            RobotContainer.armExtension.targetExtensionMeters

    if (positionError <= 0.25 &&
        rotationError.degrees.absoluteValue <= 15.0 &&
        currentVel <= 0.5 &&
        wristError.degrees.absoluteValue <= 2.0 &&
        armJointError.degrees.absoluteValue <= 5.0 &&
        armExtensionError.absoluteValue <= 0.05 &&
        currentSpeeds.omegaRadiansPerSecond.absoluteValue <= 0.25) {
      if (timer.hasElapsed(0.1)) {
        return ExecutionStatus.Success
      }
    } else {
      timer.restart()
    }

    return ExecutionStatus.Running
  }
}
