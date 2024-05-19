import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class Inverter extends DecoratorNode {
  Inverter({
    required super.position,
    super.child,
    super.uuid,
  });

  Inverter.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Inverts result';

  @override
  String get title => 'Inverter';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'inverter',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
