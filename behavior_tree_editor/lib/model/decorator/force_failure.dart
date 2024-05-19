import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class ForceFailure extends DecoratorNode {
  ForceFailure({
    required super.position,
    super.child,
    super.uuid,
  });

  ForceFailure.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Always fails';

  @override
  String get title => 'Force Failure';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'force_failure',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
