import 'package:flutter/material.dart';
import 'package:uuid/uuid.dart';

abstract class TreeNode {
  Offset position;
  List<TreeNode> children;

  final GlobalKey key;
  final String uuid;

  TreeNode({
    required this.position,
    required this.children,
    String? uuid,
  })  : key = GlobalKey(),
        uuid = uuid ?? const Uuid().v4();

  List<TreeNode> getAllNodes() {
    List<TreeNode> nodes = [this];

    for (TreeNode child in children) {
      nodes.addAll(child.getAllNodes());
    }

    return nodes;
  }

  bool get canTakeChildren;

  String get title;

  String get subTitle;

  @override
  bool operator ==(Object other) => other is TreeNode && other.key == key;

  @override
  int get hashCode => Object.hash(position, children, key);

  Map<String, dynamic> toJson();
}
