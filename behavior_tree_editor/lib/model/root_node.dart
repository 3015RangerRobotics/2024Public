import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';

class RootNode extends TreeNode {
  RootNode({
    TreeNode? child,
    required super.position,
    super.uuid,
  }) : super(
          children: [],
        ) {
    if (child != null) {
      children.add(child);
    }
  }

  RootNode.fromDataJson(Map<String, dynamic> dataJson)
      : this(
          child: nodeFromJson(dataJson['child']),
          position: offsetFromJson(dataJson['position']),
          uuid: dataJson['uuid'],
        );

  @override
  bool get canTakeChildren => children.isEmpty;

  @override
  String get title => 'Root';

  @override
  String get subTitle => 'Execution Starts Here';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'root',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
