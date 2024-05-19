package frc.robot.treeNodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.decorator.ConditionDecorator
import frc.robot.RobotContainer
import kotlin.math.absoluteValue

class DriverInputInactive(child: BehaviorTreeNode?, uuid: String) :
    ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    val x = RobotContainer.driver.leftX.absoluteValue
    val y = RobotContainer.driver.leftY.absoluteValue
    val rot = RobotContainer.driver.rightX.absoluteValue

    return x < 0.25 && y < 0.25 && rot < 0.25
  }
}
