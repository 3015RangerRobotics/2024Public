import 'package:behavior_tree_editor/model/json_util.dart';
import 'package:behavior_tree_editor/model/leaf/leaf_node.dart';

class Wait extends LeafNode {
  num waitTime;

  Wait({
    required super.position,
    this.waitTime = 0,
    super.uuid,
  });

  Wait.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          waitTime: json['time'],
          uuid: json['uuid'],
        );

  @override
  String get title => 'Wait';

  @override
  String get subTitle => '${waitTime.toStringAsFixed(2)}s';

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'wait',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'time': waitTime,
      },
    };
  }
}
