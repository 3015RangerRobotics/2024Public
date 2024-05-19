import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class Loop extends DecoratorNode {
  int loops;

  Loop({
    required super.position,
    super.child,
    this.loops = 1,
    super.uuid,
  });

  Loop.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          loops: (json['loops'] as num).toInt(),
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Repeats $loops times';

  @override
  String get title => 'Loop';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'loop',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'loops': loops,
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
