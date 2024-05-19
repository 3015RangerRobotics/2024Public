import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class InfiniteLoop extends DecoratorNode {
  InfiniteLoop({
    required super.position,
    super.child,
    super.uuid,
  });

  InfiniteLoop.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Always repeats';

  @override
  String get title => 'Infinite Loop';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'infinite_loop',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
