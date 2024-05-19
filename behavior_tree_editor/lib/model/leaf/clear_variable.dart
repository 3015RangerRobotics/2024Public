import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/leaf/leaf_node.dart';

class ClearVariable extends LeafNode {
  String variableKey;

  ClearVariable({
    required super.position,
    required this.variableKey,
    super.uuid,
  });

  ClearVariable.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          variableKey: json['variableKey'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => '<$variableKey>';

  @override
  String get title => 'Clear Variable';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'clear_var',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'variableKey': variableKey,
      },
    };
  }
}
