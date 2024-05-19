import 'package:behavior_tree_editor/model/decorator/decorator_node.dart';
import 'package:behavior_tree_editor/model/json_util.dart';

class CustomDecorator extends DecoratorNode {
  String className;

  CustomDecorator({
    required super.position,
    super.child,
    this.className = 'ClassName',
    super.uuid,
  });

  CustomDecorator.fromDataJson(Map<String, dynamic> json)
      : this(
          position: offsetFromJson(json['position']),
          className: json['className'],
          child: nodeFromJson(json['child']),
          uuid: json['uuid'],
        );

  @override
  String get subTitle => 'Custom Decorator';

  @override
  String get title => className;

  @override
  Map<String, dynamic> toJson() {
    return {
      'type': 'custom_decorator',
      'data': {
        'uuid': uuid,
        'position': position.toJson(),
        'className': className,
        'child': children.isEmpty ? null : children[0].toJson(),
      },
    };
  }
}
