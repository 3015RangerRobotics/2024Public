import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/leaf/leaf_node.dart';

class Subtree extends LeafNode {
  String? treeName;

  Subtree({
    required super.position,
    this.treeName,
    super.uuid,
  });

  Subtree.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          treeName: json['treeName'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => treeName.toString();

  @override
  String get title => 'Subtree';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'subtree',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'treeName': treeName,
      },
    };
  }
}
