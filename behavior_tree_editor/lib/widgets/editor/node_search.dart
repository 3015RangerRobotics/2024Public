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
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:behavior_tree_editor/model/var_type.dart';
import 'package:behavior_tree_editor/widgets/custom_dropdown.dart';
import 'package:flutter/material.dart';

class NodeSearch extends StatelessWidget {
  final FocusNode focusNode;
  final Function(TreeNode?) onNodeCreated;
  final Offset newNodePosition;

  const NodeSearch({
    super.key,
    required this.focusNode,
    required this.onNodeCreated,
    required this.newNodePosition,
  });

  @override
  Widget build(BuildContext context) {
    return CustomDropdownMenu<String>(
      focusNode: focusNode,
      dropdownMenuEntries: const [
        CustomDropdownMenuEntry(value: 'sequence', label: 'Sequence'),
        CustomDropdownMenuEntry(value: 'selector', label: 'Selector'),
        CustomDropdownMenuEntry(
            value: 'parallel_sequence', label: 'Parallel Sequence'),
        CustomDropdownMenuEntry(
            value: 'parallel_selector', label: 'Parallel Selector'),
        CustomDropdownMenuEntry(value: 'wait', label: 'Wait'),
        CustomDropdownMenuEntry(value: 'inverter', label: 'Inverter'),
        CustomDropdownMenuEntry(value: 'set_var', label: 'Set Variable'),
        CustomDropdownMenuEntry(value: 'check_var', label: 'Check Variable'),
        CustomDropdownMenuEntry(
            value: 'compare_vars', label: 'Compare Variables'),
        CustomDropdownMenuEntry(value: 'force_failure', label: 'Force Failure'),
        CustomDropdownMenuEntry(value: 'force_success', label: 'Force Success'),
        CustomDropdownMenuEntry(value: 'infinite_loop', label: 'Infinite Loop'),
        CustomDropdownMenuEntry(value: 'is_var_set', label: 'Is Variable Set'),
        CustomDropdownMenuEntry(
            value: 'is_var_unset', label: 'Is Variable Unset'),
        CustomDropdownMenuEntry(value: 'clear_var', label: 'Clear Variable'),
        CustomDropdownMenuEntry(value: 'loop', label: 'Loop N Times'),
        CustomDropdownMenuEntry(
            value: 'loop_until_failure', label: 'Loop Until Failure'),
        CustomDropdownMenuEntry(
            value: 'loop_until_sucess', label: 'Loop Until Success'),
        CustomDropdownMenuEntry(value: 'time_limit', label: 'Time Limit'),
        CustomDropdownMenuEntry(value: 'subtree', label: 'Subtree'),
        CustomDropdownMenuEntry(
            value: 'custom_decorator', label: 'Custom Decorator'),
        CustomDropdownMenuEntry(value: 'custom_leaf', label: 'Custom Leaf'),
      ],
      enableFilter: true,
      enableSearch: true,
      trailingIcon: const Icon(
        Icons.arrow_drop_down,
        color: Colors.transparent,
      ),
      selectedTrailingIcon: const Icon(
        Icons.arrow_drop_down,
        color: Colors.transparent,
      ),
      menuHeight: 300,
      width: 200,
      onSelected: (value) {
        if (value != null) {
          focusNode.unfocus();

          TreeNode? newNode;

          switch (value) {
            case 'sequence':
              newNode = SequenceNode(position: newNodePosition, children: []);
              break;
            case 'selector':
              newNode = SelectorNode(position: newNodePosition, children: []);
              break;
            case 'parallel_sequence':
              newNode =
                  ParallelSequence(position: newNodePosition, children: []);
              break;
            case 'parallel_selector':
              newNode =
                  ParallelSelector(position: newNodePosition, children: []);
              break;
            case 'wait':
              newNode = Wait(position: newNodePosition);
              break;
            case 'inverter':
              newNode = Inverter(position: newNodePosition);
              break;
            case 'set_var':
              newNode = SetVariable(
                  position: newNodePosition,
                  variableKey: '',
                  type: VarType.number);
              break;
            case 'check_var':
              newNode = CheckVar(
                  position: newNodePosition,
                  variableKey: '',
                  type: VarType.number);
              break;
            case 'clear_var':
              newNode = ClearVariable(
                position: newNodePosition,
                variableKey: '',
              );
              break;
            case 'compare_vars':
              newNode = CompareVars(position: newNodePosition);
              break;
            case 'force_failure':
              newNode = ForceFailure(position: newNodePosition);
              break;
            case 'force_success':
              newNode = ForceSuccess(position: newNodePosition);
              break;
            case 'infinite_loop':
              newNode = InfiniteLoop(position: newNodePosition);
              break;
            case 'is_var_set':
              newNode = IsVarSet(position: newNodePosition);
              break;
            case 'is_var_unset':
              newNode = IsVarUnset(position: newNodePosition);
              break;
            case 'loop':
              newNode = Loop(position: newNodePosition);
              break;
            case 'loop_until_failure':
              newNode = LoopUntilFailure(position: newNodePosition);
              break;
            case 'loop_until_sucess':
              newNode = LoopUntilSuccess(position: newNodePosition);
              break;
            case 'time_limit':
              newNode = TimeLimit(position: newNodePosition);
              break;
            case 'subtree':
              newNode = Subtree(position: newNodePosition);
              break;
            case 'custom_decorator':
              newNode = CustomDecorator(position: newNodePosition);
              break;
            case 'custom_leaf':
              newNode = CustomLeaf(position: newNodePosition);
              break;
          }

          onNodeCreated(newNode);
        }
      },
    );
  }
}
