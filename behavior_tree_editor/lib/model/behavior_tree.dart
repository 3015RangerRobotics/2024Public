import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/root_node.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:flutter/material.dart';

class BehaviorTree {
  String name;
  RootNode rootNode;
  List<TreeNode> detatchedNodes;

  BehaviorTree({
    required this.name,
    required this.rootNode,
    required this.detatchedNodes,
  });

  BehaviorTree.defaultTree({String name = 'New Tree'})
      : this(
          name: name,
          rootNode: RootNode(
            position: const Offset(3000, 50),
          ),
          detatchedNodes: [],
        );

  BehaviorTree.fromJson(Map<String, dynamic> json, String name)
      : this(
          name: name,
          rootNode: nodeFromJson(json['root'])! as RootNode,
          detatchedNodes: [
            for (Map<String, dynamic> node in json['detached'])
              nodeFromJson(node)!,
          ],
        );

  Map<String, dynamic> toJson() {
    return {
      'root': rootNode.toJson(),
      'detached': [for (TreeNode node in detatchedNodes) node.toJson()],
    };
  }
}
