import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class TimeLimit extends DecoratorNode {
  num timeLimit;

  TimeLimit({
    required super.position,
    super.child,
    this.timeLimit = 0,
    super.uuid,
  });

  TimeLimit.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          timeLimit: json['timeLimit'],
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get title => 'Time Limit';

  @override
  String get subTitle => '${timeLimit.toStringAsFixed(2)}s';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'time_limit',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'timeLimit': timeLimit,
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
