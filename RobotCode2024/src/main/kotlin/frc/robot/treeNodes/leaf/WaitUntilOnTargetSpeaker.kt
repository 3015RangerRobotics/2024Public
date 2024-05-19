package frc.robot.treeNodes.leaf

import com.pathplanner.lib.util.GeometryUtil
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.Timer
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.ExecutionStatus
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.robot.Constants
import frc.robot.Robot
import frc.robot.RobotContainer
import frc.robot.util.LaunchCalculator
import kotlin.math.absoluteValue

class WaitUntilOnTargetSpeaker(uuid: String) : LeafNode(uuid) {
  val timer = Timer()

  override fun handleInitialize(blackboard: Blackboard) {
    timer.restart()
  }

  override fun handleExecute(blackboard: Blackboard): ExecutionStatus {
    var targetRotation = LaunchCalculator.getCurrentLaunchState().launchAngle.toRotation2d()
    if (RobotContainer.shouldShootReversed()) {
      targetRotation += Rotation2d.fromDegrees(180.0)
    }
    val currentPose = RobotContainer.swerve.pose
    val currentRotation = currentPose.rotation
    val currentLinearVel = RobotContainer.swerve.linearVelocity
    val rotDelta = targetRotation - currentRotation

    val scoringPos =
        if (Robot.isRedAlliance()) {
          Constants.speakerPosRed
        } else {
          Constants.speakerPosBlue
        }
    val distanceToGoal = currentPose.translation.getDistance(scoringPos.toTranslation2d())

    val wristError = RobotContainer.shooterJoint.angle - RobotContainer.shooterJoint.targetAngle
    val armJointError = RobotContainer.armJoint.angle - RobotContainer.armJoint.targetAngle
    val armExtensionError =
        RobotContainer.armExtension.extensionMeters -
            RobotContainer.armExtension.targetExtensionMeters

    val shootSpeedErrorTop =
        RobotContainer.shooter.topSurfaceSpeed - RobotContainer.shooter.topTargetSurfaceSpeed
    val shootSpeedErrorBottom =
        RobotContainer.shooter.bottomSurfaceSpeed - RobotContainer.shooter.bottomTargetSurfaceSpeed

    val shootRPMErrorTop = RobotContainer.shooter.topRPM - RobotContainer.shooter.topTargetRPM
    val shootRPMErrorBottom =
        RobotContainer.shooter.bottomRPM - RobotContainer.shooter.bottomTargetRPM

    val shooterReady =
        if (RobotContainer.shooter.topTargetSurfaceSpeed != 0.0) {
          shootSpeedErrorTop.absoluteValue < 0.4 && shootSpeedErrorBottom.absoluteValue < 0.4
        } else if (RobotContainer.shooter.topTargetRPM != 0.0) {
          shootRPMErrorTop.absoluteValue < 100.0 && shootRPMErrorBottom.absoluteValue < 100.0
        } else {
          false
        }

    val rotTolerance =
        GeometryUtil.doubleLerp(5.0, 1.0, MathUtil.clamp(distanceToGoal / 8.0, 0.0, 1.0))

    if (distanceToGoal <= 6.0 &&
        rotDelta.degrees.absoluteValue <= rotTolerance &&
        wristError.degrees.absoluteValue <= 2.0 &&
        armJointError.degrees.absoluteValue <= 5.0 &&
        armExtensionError.absoluteValue <= 0.05 &&
        shooterReady) {
      if ((currentLinearVel <= 1.0 &&
          RobotContainer.localization.speakerBasedPose.isPresent &&
          timer.hasElapsed(0.1)) || timer.hasElapsed(0.2)) {
        return ExecutionStatus.Success
      }
    } else {
      timer.restart()
    }

    return ExecutionStatus.Running
  }
}
