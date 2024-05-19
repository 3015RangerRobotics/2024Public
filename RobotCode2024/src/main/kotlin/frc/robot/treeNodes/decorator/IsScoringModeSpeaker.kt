package frc.robot.treeNodes.decorator

import frc.lib.behaviorTree.Blackboard
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.decorator.ConditionDecorator
import frc.robot.Robot

class IsScoringModeSpeaker(child: BehaviorTreeNode, uuid: String) :
    ConditionDecorator(child, uuid) {
  override fun condition(blackboard: Blackboard): Boolean {
    return Robot.getScoringMode() == Robot.ScoringMode.Speaker
  }
}
