import 'dart:math';

import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';

class ArrowPainter extends StatelessWidget {
  final GlobalKey painterKey;
  final List<TreeNode> allRoots;
  final List<TreeNode> selectedNodes;
  final Offset? dragArrowPos;

  const ArrowPainter({
    super.key,
    required this.painterKey,
    required this.allRoots,
    required this.selectedNodes,
    this.dragArrowPos,
  });

  @override
  Widget build(BuildContext context) {
    return Positioned.fill(
      child: CustomPaint(
        key: painterKey,
        painter: _ArrowPainter(
          key: painterKey,
          rootNodes: allRoots,
          selectedNodes: selectedNodes,
          arrowPreviewPos: dragArrowPos,
        ),
      ),
    );
  }
}

class _ArrowPainter extends CustomPainter {
  final GlobalKey key;
  final List<TreeNode> rootNodes;
  final List<TreeNode> selectedNodes;
  final Offset? arrowPreviewPos;

  const _ArrowPainter({
    required this.key,
    required this.rootNodes,
    required this.selectedNodes,
    this.arrowPreviewPos,
  });

  @override
  void paint(Canvas canvas, Size size) {
    for (TreeNode node in rootNodes) {
      _paintNodeArrows(canvas, node);
    }

    if (selectedNodes.length == 1 && arrowPreviewPos != null) {
      Size selectedSize = selectedNodes[0].key.currentContext!.size!;
      Offset selectedPos = (selectedNodes[0]
              .key
              .currentContext!
              .findRenderObject() as RenderBox)
          .localToGlobal(Offset(selectedSize.width / 2, selectedSize.height));
      Offset selectedArrowStart =
          (key.currentContext!.findRenderObject() as RenderBox)
              .globalToLocal(selectedPos);

      _drawArrow(canvas, selectedArrowStart, arrowPreviewPos!);
    }
  }

  void _paintNodeArrows(Canvas canvas, TreeNode root) {
    Size rootSize = root.key.currentContext!.size!;
    Offset rootPos = (root.key.currentContext!.findRenderObject() as RenderBox)
        .localToGlobal(Offset(rootSize.width / 2, rootSize.height));
    Offset rootArrowStart =
        (key.currentContext!.findRenderObject() as RenderBox)
            .globalToLocal(rootPos);

    for (TreeNode child in root.children) {
      Size childSize = child.key.currentContext!.size!;
      Offset childPos =
          (child.key.currentContext!.findRenderObject() as RenderBox)
              .localToGlobal(Offset(childSize.width / 2, 0));
      Offset childArrowEnd =
          (key.currentContext!.findRenderObject() as RenderBox)
              .globalToLocal(childPos);

      _drawArrow(canvas, rootArrowStart, childArrowEnd);

      _paintNodeArrows(canvas, child);
    }
  }

  void _drawArrow(Canvas canvas, Offset start, Offset end) {
    Paint p = Paint()
      ..color = Colors.grey[700]!
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    const arrowSize = 12.0;
    const arrowAngle = 25 * pi / 180;

    final angle = (end - start).direction;
    Offset lineEnd = end - Offset.fromDirection(angle, arrowSize - 2);

    canvas.drawLine(start, lineEnd, p);

    p.style = PaintingStyle.fill;

    final path = Path();

    path.moveTo(end.dx - arrowSize * cos(angle - arrowAngle),
        end.dy - arrowSize * sin(angle - arrowAngle));
    path.lineTo(end.dx, end.dy);
    path.lineTo(end.dx - arrowSize * cos(angle + arrowAngle),
        end.dy - arrowSize * sin(angle + arrowAngle));
    path.close();
    canvas.drawPath(path, p);
  }

  @override
  bool shouldRepaint(_ArrowPainter oldDelegate) {
    return listEquals(oldDelegate.rootNodes, rootNodes) ||
        listEquals(oldDelegate.selectedNodes, selectedNodes) ||
        oldDelegate.arrowPreviewPos != arrowPreviewPos;
  }
}
