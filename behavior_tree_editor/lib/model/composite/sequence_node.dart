import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';

class SequenceNode extends TreeNode {
  SequenceNode({
    required super.position,
    required super.children,
    super.uuid,
  });

  SequenceNode.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          children: [
            for (Map<String, dynamic> child in json['children'])
              nodeFromJson(child)!,
          ],
          uuid: json['uuid'],
        );

  @override
  bool get canTakeChildren => true;

  @override
  String get title => 'Sequence';

  @override
  String get subTitle => 'Succeeds on all success';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'sequence',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'children': [
          for (TreeNode child in orderedChildren(children)) child.toJson()
        ],
      },
    };
  }
}
