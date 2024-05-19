import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/leaf/leaf_node.dart';

class CustomLeaf extends LeafNode {
  String className;

  CustomLeaf({
    required super.position,
    this.className = 'ClassName',
    super.uuid,
  });

  CustomLeaf.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          className: json['className'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Custom Leaf';

  @override
  String get title => className;

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'custom_leaf',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'className': className,
      },
    };
  }
}
