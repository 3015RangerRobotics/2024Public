import 'package:behavior_tree_editor/model/decorator/check_var.dart';
import 'package:behavior_tree_editor/model/decorator/compare_vars.dart';
import 'package:behavior_tree_editor/model/decorator/custom_decorator.dart';
import 'package:behavior_tree_editor/model/decorator/is_var_set.dart';
import 'package:behavior_tree_editor/model/decorator/is_var_unset.dart';
import 'package:behavior_tree_editor/model/decorator/loop.dart';
import 'package:behavior_tree_editor/model/decorator/time_limit.dart';
import 'package:behavior_tree_editor/model/leaf/clear_variable.dart';
import 'package:behavior_tree_editor/model/leaf/custom_leaf.dart';
import 'package:behavior_tree_editor/model/leaf/set_variable.dart';
import 'package:behavior_tree_editor/model/leaf/subtree.dart';
import 'package:behavior_tree_editor/model/leaf/wait.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:behavior_tree_editor/model/var_type.dart';
import 'package:behavior_tree_editor/widgets/number_text_field.dart';
import 'package:behavior_tree_editor/widgets/string_text_field.dart';
import 'package:flutter/material.dart';

class DetailsPanel extends StatelessWidget {
  final double width;
  final TreeNode? selectedNode;
  final VoidCallback onNodeChanged;

  const DetailsPanel({
    super.key,
    this.width = 400,
    required this.selectedNode,
    required this.onNodeChanged,
  });

  @override
  Widget build(BuildContext context) {
    return Card(
      elevation: 4,
      margin: const EdgeInsets.all(0),
      child: SizedBox(
        width: width,
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: Column(
            mainAxisSize: MainAxisSize.max,
            mainAxisAlignment: MainAxisAlignment.start,
            crossAxisAlignment: CrossAxisAlignment.start,
            children: [
              if (selectedNode != null)
                Text(
                  selectedNode!.title,
                  style: const TextStyle(fontSize: 20),
                ),
              if (selectedNode != null) const Divider(),
              if (selectedNode != null) const SizedBox(height: 8),
              ..._getDetailsForSelected(),
            ],
          ),
        ),
      ),
    );
  }

  List<Widget> _getDetailsForSelected() {
    return switch (selectedNode?.runtimeType) {
      Wait => _waitDetails(),
      ClearVariable => _clearVarDetails(),
      SetVariable => _setVarDetails(),
      CheckVar => _checkVarDetails(),
      CompareVars => _compareVarsDetails(),
      IsVarSet => _isVarSetDetails(),
      IsVarUnset => _isVarUnsetDetails(),
      Loop => _loopDetails(),
      TimeLimit => _timeLimitDetails(),
      Subtree => _subtreeDetails(),
      CustomDecorator => _customDecoratorDetails(),
      CustomLeaf => _customLeafDetails(),
      _ => [],
    };
  }

  List<Widget> _customDecoratorDetails() {
    CustomDecorator node = selectedNode as CustomDecorator;

    return [
      StringTextField(
        initialText: node.className,
        label: 'Class Name',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.className = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _customLeafDetails() {
    CustomLeaf node = selectedNode as CustomLeaf;

    return [
      StringTextField(
        initialText: node.className,
        label: 'Class Name',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.className = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _subtreeDetails() {
    Subtree node = selectedNode as Subtree;

    return [
      StringTextField(
        initialText: node.treeName ?? '',
        label: 'Tree Name',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.treeName = value;
          } else {
            node.treeName = null;
          }
          onNodeChanged();
        },
      ),
    ];
  }

  List<Widget> _timeLimitDetails() {
    TimeLimit node = selectedNode as TimeLimit;

    return [
      NumberTextField(
        initialText: node.timeLimit.toStringAsFixed(2),
        label: 'Time Limit',
        arrowKeyIncrement: 0.05,
        onSubmitted: (value) {
          if (value != null) {
            node.timeLimit = value;
            onNodeChanged();
          }
        },
      )
    ];
  }

  List<Widget> _loopDetails() {
    Loop node = selectedNode as Loop;

    return [
      NumberTextField(
        initialText: node.loops.toString(),
        label: 'Loop Count',
        arrowKeyIncrement: 1,
        onSubmitted: (value) {
          if (value != null) {
            node.loops = value.round();
            onNodeChanged();
          }
        },
      )
    ];
  }

  List<Widget> _isVarSetDetails() {
    IsVarSet node = selectedNode as IsVarSet;

    return [
      StringTextField(
        initialText: node.variableKey,
        label: 'Variable Key',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.variableKey = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _isVarUnsetDetails() {
    IsVarUnset node = selectedNode as IsVarUnset;

    return [
      StringTextField(
        initialText: node.variableKey,
        label: 'Variable Key',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.variableKey = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _compareVarsDetails() {
    CompareVars node = selectedNode as CompareVars;

    return [
      StringTextField(
        initialText: node.varKeyA,
        label: 'Variable Key A',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.varKeyA = value;
            onNodeChanged();
          }
        },
      ),
      const SizedBox(height: 12),
      StringTextField(
        initialText: node.varKeyB,
        label: 'Variable Key B',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.varKeyB = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _waitDetails() {
    Wait node = selectedNode as Wait;

    return [
      NumberTextField(
        initialText: node.waitTime.toStringAsFixed(2),
        label: 'Wait Time',
        arrowKeyIncrement: 0.05,
        onSubmitted: (value) {
          if (value != null) {
            node.waitTime = value;
          }
          onNodeChanged();
        },
      ),
    ];
  }

  List<Widget> _clearVarDetails() {
    ClearVariable node = selectedNode as ClearVariable;

    return [
      StringTextField(
        initialText: node.variableKey,
        label: 'Variable Key',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.variableKey = value;
            onNodeChanged();
          }
        },
      ),
    ];
  }

  List<Widget> _setVarDetails() {
    SetVariable node = selectedNode as SetVariable;

    return [
      StringTextField(
        initialText: node.variableKey,
        label: 'Variable Key',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.variableKey = value;
            onNodeChanged();
          }
        },
      ),
      const SizedBox(height: 12),
      DropdownMenu<VarType>(
        dropdownMenuEntries: const [
          DropdownMenuEntry(value: VarType.number, label: 'Number'),
          DropdownMenuEntry(value: VarType.string, label: 'String'),
          DropdownMenuEntry(value: VarType.boolean, label: 'Boolean'),
        ],
        width: width - 16,
        enableFilter: false,
        enableSearch: false,
        initialSelection: node.type,
        label: const Text('Variable Type'),
        inputDecorationTheme: InputDecorationTheme(
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(8),
          ),
          contentPadding: const EdgeInsets.fromLTRB(12, 0, 12, 0),
          isDense: true,
          constraints: const BoxConstraints(
            maxHeight: 44,
          ),
        ),
        onSelected: (value) {
          if (value != null) {
            if (value != node.type) {
              if (value == VarType.string) {
                node.value = '';
              } else if (value == VarType.number) {
                node.value = 0;
              } else if (value == VarType.boolean) {
                node.value = false;
              }
            }
            node.type = value;
            onNodeChanged();
          }
        },
      ),
      const SizedBox(height: 12),
      switch (node.type) {
        VarType.number => NumberTextField(
            initialText: node.value.toString(),
            label: 'Value',
            arrowKeyIncrement: 0.01,
            onSubmitted: (value) {
              if (value != null) {
                node.value = value;
                onNodeChanged();
              }
            },
          ),
        VarType.string => StringTextField(
            initialText: node.value.toString(),
            label: 'Value',
            onSubmitted: (value) {
              node.value = value;
              onNodeChanged();
            },
          ),
        VarType.boolean => Row(
            children: [
              Checkbox(
                value: node.value,
                onChanged: (value) {
                  node.value = value;
                  onNodeChanged();
                },
              ),
              const Text(
                'Value',
                style: TextStyle(fontSize: 18),
              ),
            ],
          ),
      }
    ];
  }

  List<Widget> _checkVarDetails() {
    CheckVar node = selectedNode as CheckVar;

    return [
      StringTextField(
        initialText: node.variableKey,
        label: 'Variable Key',
        onSubmitted: (value) {
          if (value.isNotEmpty) {
            node.variableKey = value;
            onNodeChanged();
          }
        },
      ),
      const SizedBox(height: 12),
      DropdownMenu<VarType>(
        dropdownMenuEntries: const [
          DropdownMenuEntry(value: VarType.number, label: 'Number'),
          DropdownMenuEntry(value: VarType.string, label: 'String'),
          DropdownMenuEntry(value: VarType.boolean, label: 'Boolean'),
        ],
        width: width - 16,
        enableFilter: false,
        enableSearch: false,
        initialSelection: node.type,
        label: const Text('Variable Type'),
        inputDecorationTheme: InputDecorationTheme(
          border: OutlineInputBorder(
            borderRadius: BorderRadius.circular(8),
          ),
          contentPadding: const EdgeInsets.fromLTRB(12, 0, 12, 0),
          isDense: true,
          constraints: const BoxConstraints(
            maxHeight: 44,
          ),
        ),
        onSelected: (value) {
          if (value != null) {
            if (value != node.type) {
              if (value == VarType.string) {
                node.value = '';
              } else if (value == VarType.number) {
                node.value = 0;
              } else if (value == VarType.boolean) {
                node.value = false;
              }
            }
            node.type = value;
            onNodeChanged();
          }
        },
      ),
      const SizedBox(height: 12),
      switch (node.type) {
        VarType.number => NumberTextField(
            initialText: node.value.toString(),
            label: 'Value',
            arrowKeyIncrement: 0.01,
            onSubmitted: (value) {
              if (value != null) {
                node.value = value;
                onNodeChanged();
              }
            },
          ),
        VarType.string => StringTextField(
            initialText: node.value.toString(),
            label: 'Value',
            onSubmitted: (value) {
              node.value = value;
              onNodeChanged();
            },
          ),
        VarType.boolean => Row(
            children: [
              Checkbox(
                value: node.value,
                onChanged: (value) {
                  node.value = value;
                  onNodeChanged();
                },
              ),
              const Text(
                'Value',
                style: TextStyle(fontSize: 18),
              ),
            ],
          ),
      }
    ];
  }
}
