import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class ForceSuccess extends DecoratorNode {
  ForceSuccess({
    required super.position,
    super.child,
    super.uuid,
  });

  ForceSuccess.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Always succeeds';

  @override
  String get title => 'Force Success';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'force_success',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
