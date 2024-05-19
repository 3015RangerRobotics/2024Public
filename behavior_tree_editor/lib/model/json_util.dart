import 'package:behavior_tree_editor/model/composite/parallel_selector.dart';
import 'package:behavior_tree_editor/model/composite/parallel_sequence.dart';
import 'package:behavior_tree_editor/model/composite/selector_node.dart';
import 'package:behavior_tree_editor/model/composite/sequence_node.dart';
import 'package:behavior_tree_editor/model/decorator/check_var.dart';
import 'package:behavior_tree_editor/model/decorator/compare_vars.dart';
import 'package:behavior_tree_editor/model/decorator/custom_decorator.dart';
import 'package:behavior_tree_editor/model/decorator/force_failure.dart';
import 'package:behavior_tree_editor/model/decorator/force_success.dart';
import 'package:behavior_tree_editor/model/decorator/infinite_loop.dart';
import 'package:behavior_tree_editor/model/decorator/inverter.dart';
import 'package:behavior_tree_editor/model/decorator/is_var_set.dart';
import 'package:behavior_tree_editor/model/decorator/is_var_unset.dart';
import 'package:behavior_tree_editor/model/decorator/loop.dart';
import 'package:behavior_tree_editor/model/decorator/loop_until_failure.dart';
import 'package:behavior_tree_editor/model/decorator/loop_until_success.dart';
import 'package:behavior_tree_editor/model/decorator/time_limit.dart';
import 'package:behavior_tree_editor/model/leaf/clear_variable.dart';
import 'package:behavior_tree_editor/model/leaf/custom_leaf.dart';
import 'package:behavior_tree_editor/model/leaf/set_variable.dart';
import 'package:behavior_tree_editor/model/leaf/subtree.dart';
import 'package:behavior_tree_editor/model/leaf/wait.dart';
import 'package:behavior_tree_editor/model/root_node.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:flutter/material.dart';

TreeNode? nodeFromJson(Map<String, dynamic>? json) {
  if (json == null) return null;

  return switch (json['type']) {
    'root' => RootNode.fromDataJson(json['data']),
    'wait' => Wait.fromDataJson(json['data']),
    'set_var' => SetVariable.fromDataJson(json['data']),
    'clear_var' => ClearVariable.fromDataJson(json['data']),
    'check_var' => CheckVar.fromDataJson(json['data']),
    'selector' => SelectorNode.fromDataJson(json['data']),
    'parallel_selector' => ParallelSelector.fromDataJson(json['data']),
    'sequence' => SequenceNode.fromDataJson(json['data']),
    'parallel_sequence' => ParallelSequence.fromDataJson(json['data']),
    'time_limit' => TimeLimit.fromDataJson(json['data']),
    'loop' => Loop.fromDataJson(json['data']),
    'loop_until_success' => LoopUntilSuccess.fromDataJson(json['data']),
    'loop_until_failure' => LoopUntilFailure.fromDataJson(json['data']),
    'is_var_unset' => IsVarUnset.fromDataJson(json['data']),
    'is_var_set' => IsVarSet.fromDataJson(json['data']),
    'inverter' => Inverter.fromDataJson(json['data']),
    'infinite_loop' => InfiniteLoop.fromDataJson(json['data']),
    'force_success' => ForceSuccess.fromDataJson(json['data']),
    'force_failure' => ForceFailure.fromDataJson(json['data']),
    'compare_vars' => CompareVars.fromDataJson(json['data']),
    'subtree' => Subtree.fromDataJson(json['data']),
    'custom_decorator' => CustomDecorator.fromDataJson(json['data']),
    'custom_leaf' => CustomLeaf.fromDataJson(json['data']),
    _ => null,
  };
}

Offset offsetFromJson(Map<String, dynamic> offsetJson) {
  return Offset(
      (offsetJson['x'] as num).toDouble(), (offsetJson['y'] as num).toDouble());
}

List<TreeNode> orderedChildren(List<TreeNode> children) {
  List<TreeNode> ordered = [...children];
  ordered.sort((a, b) => a.position.dx.compareTo(b.position.dx));

  return ordered;
}

extension ToJSON on Offset {
  Map<String, dynamic> toJson() {
    return {
      'x': dx,
      'y': dy,
    };
  }
}
