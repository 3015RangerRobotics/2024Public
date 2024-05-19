import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class IsVarSet extends DecoratorNode {
  String variableKey;

  IsVarSet({
    required super.position,
    super.child,
    this.variableKey = '',
    super.uuid,
  });

  IsVarSet.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          variableKey: json['variableKey'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Only runs if <$variableKey> is set';

  @override
  String get title => 'Is Variable Set';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'is_var_set',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
        'variableKey': variableKey,
      },
    };
  }
}
