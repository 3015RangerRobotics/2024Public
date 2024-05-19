import 'package:behavior_tree_editor/model/tree_node.dart';

abstract class DecoratorNode extends TreeNode {
  DecoratorNode({
    required super.position,
    TreeNode? child,
    super.uuid,
  }) : super(children: child == null ? [] : [child]);

  @override
  bool get canTakeChildren => children.isEmpty;
}
