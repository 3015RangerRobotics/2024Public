import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/leaf/leaf_node.dart';
import 'package:behavior_tree_editor/model/var_type.dart';

class SetVariable extends LeafNode {
  String variableKey;
  VarType type;
  dynamic value;

  SetVariable({
    required super.position,
    required this.variableKey,
    required this.type,
    this.value,
    super.uuid,
  });

  SetVariable.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          variableKey: json['variableKey'],
          type: varTypeFromString(json['variableType']),
          value: json['value'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => switch (type) {
        VarType.number || VarType.boolean => '<$variableKey> = $value',
        VarType.string => '<$variableKey> = "$value"',
      };

  @override
  String get title => 'Set Variable';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'set_var',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'variableKey': variableKey,
        'variableType': type.name,
        'value': value,
      },
    };
  }
}
