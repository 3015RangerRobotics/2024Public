package frc.robot.treeNodes.decorator

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.decorator.ConditionDecorator
import frc.robot.RobotContainer
import frc.robot.subsystems.lidar.LidarDetection
import kotlin.math.absoluteValue
import kotlin.math.asin
import kotlin.math.hypot

class PathClear(child: BehaviorTreeNode?, uuid: String) : ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    //    val robots = RobotContainer.lidar.currentRobotDetections
    val robots: List<LidarDetection> = ArrayList()

    if (robots.isEmpty()) {
      return true
    }

    val robotPose = RobotContainer.swerve.pose

    val currentSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            RobotContainer.swerve.currentSpeeds, robotPose.rotation)
    val directionOfTravel =
        Rotation2d(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    val currentVel = hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond)
    val stoppingDistance = ((currentVel * currentVel) / (2.0 * 6.0)) + 1.0

    val dangerZoneAngle = Rotation2d(asin(0.75 / stoppingDistance))

    val goalPose = blackboard.getPose2d("PathfindingGoalPose") ?: return true

    val angleToGoal = (goalPose.translation - robotPose.translation).angle

    for (robot in robots) {
      val angleToDet = (robot.boundingBoxCenter.toTranslation2d() - robotPose.translation).angle

      val distance = robot.boundingBoxCenter.toTranslation2d().getDistance(robotPose.translation)

      if ((angleToDet - directionOfTravel).degrees.absoluteValue <
          dangerZoneAngle.degrees.absoluteValue &&
          distance <= stoppingDistance &&
          distance >= 0.5) {
        return false
      }

      if ((angleToDet - angleToGoal).degrees.absoluteValue <= 90 &&
          distance <= 0.8 &&
          distance >= 0.5) {
        return false
      }
    }

    return true
  }
}
