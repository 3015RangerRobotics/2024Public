import 'package:behavior_tree_editor/model/tree_node.dart';

abstract class LeafNode extends TreeNode {
  LeafNode({
    required super.position,
    super.uuid,
  }) : super(children: []);

  @override
  bool get canTakeChildren => false;
}
