import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class CompareVars extends DecoratorNode {
  String varKeyA;
  String varKeyB;

  CompareVars({
    required super.position,
    super.child,
    this.varKeyA = '',
    this.varKeyB = '',
    super.uuid,
  });

  CompareVars.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          varKeyA: json['variableKeyA'],
          varKeyB: json['variableKeyB'],
          uuid: json['uuid'],
        );

  @override
  String get subTitle => '<$varKeyA> == <$varKeyB>';

  @override
  String get title => 'Compare Variables';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'compare_vars',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
        'variableKeyA': varKeyA,
        'variableKeyB': varKeyB,
      },
    };
  }
}
