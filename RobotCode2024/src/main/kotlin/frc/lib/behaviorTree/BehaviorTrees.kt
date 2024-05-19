package frc.lib.behaviorTree

import edu.wpi.first.wpilibj.Filesystem
import frc.lib.behaviorTree.nodes.BehaviorTreeNode
import frc.lib.behaviorTree.nodes.Subtree
import frc.lib.behaviorTree.nodes.composite.ParallelSelector
import frc.lib.behaviorTree.nodes.composite.ParallelSequence
import frc.lib.behaviorTree.nodes.composite.Selector
import frc.lib.behaviorTree.nodes.composite.Sequence
import frc.lib.behaviorTree.nodes.decorator.CheckVar
import frc.lib.behaviorTree.nodes.decorator.CompareVars
import frc.lib.behaviorTree.nodes.decorator.DecoratorNode
import frc.lib.behaviorTree.nodes.decorator.ForceFailure
import frc.lib.behaviorTree.nodes.decorator.ForceSuccess
import frc.lib.behaviorTree.nodes.decorator.InfiniteLoop
import frc.lib.behaviorTree.nodes.decorator.Inverter
import frc.lib.behaviorTree.nodes.decorator.IsVarSet
import frc.lib.behaviorTree.nodes.decorator.IsVarUnset
import frc.lib.behaviorTree.nodes.decorator.Loop
import frc.lib.behaviorTree.nodes.decorator.LoopUntilFailure
import frc.lib.behaviorTree.nodes.decorator.LoopUntilSuccess
import frc.lib.behaviorTree.nodes.decorator.TimeLimit
import frc.lib.behaviorTree.nodes.leaf.ClearVariable
import frc.lib.behaviorTree.nodes.leaf.LeafNode
import frc.lib.behaviorTree.nodes.leaf.SetVariable
import frc.lib.behaviorTree.nodes.leaf.Wait
import java.io.File
import org.json.simple.JSONArray
import org.json.simple.JSONObject
import org.json.simple.parser.JSONParser

class BehaviorTrees {
  companion object Companion {
    @JvmStatic
    fun loadTree(treeName: String): BehaviorTreeNode? {
      val fileContent =
          File(Filesystem.getDeployDirectory(), "behavior_trees/$treeName.tree").readText()
      val json = JSONParser().parse(fileContent) as JSONObject

      // Skip the "root" node. It's not needed
      val rootJson = json["root"] as JSONObject
      val dataJson = rootJson["data"] as JSONObject
      val childJson = dataJson["child"] as JSONObject

      return nodeFromJson(childJson)
    }

    private fun nodeFromJson(json: JSONObject?): BehaviorTreeNode? {
      if (json == null) {
        return null
      }

      val type = json["type"] as String
      val data = json["data"] as JSONObject
      val uuid = data["uuid"] as String

      return when (type) {
        "selector" -> Selector(getChildren(data), uuid)
        "parallel_selector" -> ParallelSelector(getChildren(data), uuid)
        "sequence" -> Sequence(getChildren(data), uuid)
        "parallel_sequence" -> ParallelSequence(getChildren(data), uuid)
        "wait" -> Wait((data["time"] as Number).toDouble(), uuid)
        "subtree" -> Subtree(loadTree(data["treeName"] as String), uuid)
        "set_var" -> SetVariable(data["variableKey"] as String, data["value"], uuid)
        "check_var" ->
            CheckVar(
                nodeFromJson(data["child"] as JSONObject?),
                data["variableKey"] as String,
                data["value"],
                uuid)
        "custom_leaf" -> {
          val leafClass = Class.forName("frc.robot.treeNodes.leaf." + data["className"] as String)
          val leafConstructor = leafClass.getConstructor(String::class.java)
          leafConstructor.newInstance(uuid) as LeafNode
        }
        "clear_var" -> ClearVariable(data["variableKey"] as String, uuid)
        "time_limit" ->
            TimeLimit(
                nodeFromJson(data["child"] as JSONObject?),
                (data["timeLimit"] as Number).toDouble(),
                uuid)
        "loop" ->
            Loop(
                nodeFromJson(data["child"] as JSONObject?), (data["loops"] as Number).toInt(), uuid)
        "loop_until_success" -> LoopUntilSuccess(nodeFromJson(data["child"] as JSONObject), uuid)
        "loop_until_failure" -> LoopUntilFailure(nodeFromJson(data["child"] as JSONObject), uuid)
        "is_var_unset" ->
            IsVarUnset(
                nodeFromJson(data["child"] as JSONObject?), data["variableKey"] as String, uuid)
        "is_var_set" ->
            IsVarSet(
                nodeFromJson(data["child"] as JSONObject?), data["variableKey"] as String, uuid)
        "inverter" -> Inverter(nodeFromJson(data["child"] as JSONObject?), uuid)
        "infinite_loop" -> InfiniteLoop(nodeFromJson(data["child"] as JSONObject?), uuid)
        "force_success" -> ForceSuccess(nodeFromJson(data["child"] as JSONObject?), uuid)
        "force_failure" -> ForceFailure(nodeFromJson(data["child"] as JSONObject?), uuid)
        "custom_decorator" -> {
          val decoratorClass =
              Class.forName("frc.robot.treeNodes.decorator." + data["className"] as String)
          val decoratorConstructor =
              decoratorClass.getConstructor(BehaviorTreeNode::class.java, String::class.java)
          decoratorConstructor.newInstance(nodeFromJson(data["child"] as JSONObject?), uuid)
              as DecoratorNode
        }
        "compare_vars" ->
            CompareVars(
                nodeFromJson(data["child"] as JSONObject?),
                data["variableKeyA"] as String,
                data["variableKeyB"] as String,
                uuid)
        else -> throw RuntimeException("Unknown node type: $type")
      }
    }

    private fun getChildren(parentData: JSONObject): List<BehaviorTreeNode> {
      val children =
          (parentData["children"] as JSONArray).mapNotNull { nodeFromJson(it as JSONObject) }

      return children
    }
  }
}
