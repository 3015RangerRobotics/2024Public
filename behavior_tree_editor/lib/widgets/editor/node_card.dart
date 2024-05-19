import 'package:behavior_tree_editor/model/leaf/subtree.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:flutter/material.dart';

class NodeCard extends StatelessWidget {
  final TreeNode node;
  final double width;
  final double dragAreaWidth;
  final double dragAreaHeight;
  final bool selected;
  final bool raised;
  final bool active;

  const NodeCard({
    super.key,
    required this.node,
    this.width = 200,
    this.dragAreaWidth = 125,
    this.dragAreaHeight = 25,
    this.selected = false,
    this.raised = false,
    this.active = false,
  });

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    Color? color;
    if (selected) {
      color = colorScheme.primaryContainer;
    } else if (active) {
      color = colorScheme.primary;
    } else if (node is Subtree) {
      color = colorScheme.surfaceVariant;
    }

    Color? dividerColor;
    if (selected) {
      dividerColor = colorScheme.onPrimaryContainer.withOpacity(0.5);
    } else if (active) {
      dividerColor = colorScheme.onPrimary.withOpacity(0.5);
    } else if (node is Subtree) {
      dividerColor = colorScheme.onSurfaceVariant.withOpacity(0.5);
    }

    Color? textColor;
    if (selected) {
      textColor = colorScheme.onPrimaryContainer;
    } else if (active) {
      textColor = colorScheme.onPrimary;
    } else if (node is Subtree) {
      textColor = colorScheme.onSurfaceVariant;
    }

    return Card(
      key: node.key,
      elevation: raised ? 8 : 2,
      color: color,
      margin: const EdgeInsets.all(0),
      child: SizedBox(
        width: width,
        height: 130,
        child: Stack(
          children: [
            Positioned.fill(
              child: Column(
                mainAxisAlignment: MainAxisAlignment.start,
                crossAxisAlignment: CrossAxisAlignment.center,
                children: [
                  Padding(
                    padding: const EdgeInsets.all(8.0),
                    child: FittedBox(
                      fit: BoxFit.scaleDown,
                      child: Text(
                        node.title,
                        style: TextStyle(
                          fontSize: 28,
                          color: textColor,
                        ),
                      ),
                    ),
                  ),
                  Divider(
                    height: 2,
                    color: dividerColor,
                  ),
                  if (node.subTitle.isNotEmpty)
                    Padding(
                      padding: const EdgeInsets.all(8.0),
                      child: FittedBox(
                        fit: BoxFit.scaleDown,
                        child: Text(
                          node.subTitle,
                          style: TextStyle(
                            fontSize: 22,
                            color: textColor?.withOpacity(0.5) ?? Colors.grey,
                          ),
                        ),
                      ),
                    ),
                ],
              ),
            ),
            if (node.canTakeChildren)
              Align(
                alignment: Alignment.bottomCenter,
                child: Container(
                  width: dragAreaWidth,
                  height: dragAreaHeight,
                  color: Colors.grey[800],
                ),
              ),
          ],
        ),
      ),
    );
  }
}
