import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/var_type.dart';

class CheckVar extends DecoratorNode {
  String variableKey;
  VarType type;
  dynamic value;

  CheckVar({
    required super.position,
    super.child,
    this.variableKey = '',
    required this.type,
    this.value,
    super.uuid,
  });

  CheckVar.fromDataJson(Map<String, dynamic> json)
      : this(
          child: nodeFromJson(json['child']),
          position: offsetFromJson(json['position']),
          variableKey: json['variableKey'],
          type: varTypeFromString(json['variableType']),
          value: json['value'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => '<$variableKey> == $value';

  @override
  String get title => 'Check Variable';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'check_var',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
        'variableKey': variableKey,
        'variableType': type.name,
        'value': value,
      },
    };
  }
}
