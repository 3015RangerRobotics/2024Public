import 'dart:collection';
import 'dart:convert';
import 'dart:io';
import 'dart:math';

import 'package:behavior_tree_editor/model/behavior_tree.dart';
import 'package:behavior_tree_editor/model/root_node.dart';
import 'package:behavior_tree_editor/model/tree_node.dart';
import 'package:behavior_tree_editor/pages/welcome_page.dart';
import 'package:behavior_tree_editor/services/debug_state.dart';
import 'package:behavior_tree_editor/widgets/custom_appbar.dart';
import 'package:behavior_tree_editor/widgets/editor/arrow_painter.dart';
import 'package:behavior_tree_editor/widgets/editor/details_panel.dart';
import 'package:behavior_tree_editor/widgets/editor/node_card.dart';
import 'package:behavior_tree_editor/widgets/editor/node_search.dart';
import 'package:behavior_tree_editor/widgets/keyboard_shortcuts.dart';
import 'package:behavior_tree_editor/widgets/renamable_title.dart';
import 'package:dotted_border/dotted_border.dart';
import 'package:file/local.dart';
import 'package:file_selector/file_selector.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:macos_secure_bookmarks/macos_secure_bookmarks.dart';
import 'package:path/path.dart';
import 'package:shared_preferences/shared_preferences.dart';

class EditorPage extends StatefulWidget {
  final SharedPreferences prefs;

  const EditorPage({
    super.key,
    required this.prefs,
  });

  @override
  State<EditorPage> createState() => _EditorPageState();
}

class _EditorPageState extends State<EditorPage> {
  static const double _canvasWidth = 8000;
  static const double _canvasHeight = 6000;
  static const double _canvasPadding = 5000;
  static const double _nodeWidth = 300;
  static const double _nodeDragAreaWidth = 125;
  static const double _nodeDragAreaHeight = 25;

  Directory? _projectDir;
  late Directory _deployDir;
  final SecureBookmarks? _bookmarks =
      Platform.isMacOS ? SecureBookmarks() : null;
  final fs = const LocalFileSystem();

  final TransformationController _controller = TransformationController();
  List<TreeNode> _selectedNodes = [];
  TreeNode? _raisedNode;
  Offset? _dragArrowPos;
  Offset? _addMenuPos;
  final FocusNode _addMenuFocusNode = FocusNode();

  final GlobalKey _paintKey = GlobalKey();
  final List<BehaviorTree> _trees = [];
  late BehaviorTree _currentTree;
  int _treeIdx = 0;
  Offset? _selectionRectA;
  Offset? _selectionRectB;

  final DebugState _debugState = DebugState();
  bool _connected = false;
  HashSet<String> _activeNodes = HashSet();

  @override
  void initState() {
    super.initState();

    const double initialZoom = 0.8;

    // Zoom
    _controller.value.setEntry(0, 0, initialZoom);
    _controller.value.setEntry(1, 1, initialZoom);
    _controller.value.setEntry(2, 2, initialZoom);

    // X translation
    _controller.value.setEntry(0, 3, -2500);

    _openProject();

    _debugState.connectionStatusStream().listen((event) {
      setState(() {
        _connected = event;
        _activeNodes.clear();
      });
    });

    _debugState.activeNodesStream().listen((event) {
      setState(() {
        _activeNodes = event;
      });
    });
  }

  @override
  Widget build(BuildContext context) {
    if (_projectDir == null) return Container();

    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Scaffold(
      appBar: CustomAppBar(
        titleWidget: Text(_currentTree.name),
      ),
      drawer: Stack(
        children: [
          NavigationDrawer(
            selectedIndex: _treeIdx,
            onDestinationSelected: (value) {
              setState(() {
                _treeIdx = value;
                _currentTree = _trees[value];
                _selectedNodes.clear();
                _raisedNode = null;
                _dragArrowPos = null;
                _addMenuPos = null;
              });
            },
            children: [
              DrawerHeader(
                child: Center(
                  child: Column(
                    mainAxisAlignment: MainAxisAlignment.center,
                    children: [
                      Expanded(
                        flex: 2,
                        child: Container(),
                      ),
                      Padding(
                        padding: const EdgeInsets.all(8.0),
                        child: Text(
                          basename(_projectDir!.path),
                          style: const TextStyle(
                            fontSize: 20,
                          ),
                        ),
                      ),
                      ElevatedButton(
                        style: ElevatedButton.styleFrom(
                          foregroundColor: colorScheme.onPrimaryContainer,
                          backgroundColor: colorScheme.primaryContainer,
                        ),
                        onPressed: () {
                          _openProjectDialog(context);
                        },
                        child: const Text('Switch Project'),
                      ),
                      Builder(
                        builder: (context) {
                          if (_connected) {
                            return const Text(
                              'Connected',
                              style: TextStyle(color: Colors.green),
                            );
                          } else {
                            return const Text(
                              'Disconnected',
                              style: TextStyle(color: Colors.red),
                            );
                          }
                        },
                      ),
                      Expanded(
                        flex: 4,
                        child: Container(),
                      ),
                    ],
                  ),
                ),
              ),
              for (int i = 0; i < _trees.length; i++)
                NavigationDrawerDestination(
                  icon: Container(),
                  label: RenamableTitle(
                    title: _trees[i].name,
                    onRename: (value) {
                      _renameTree(i, value);
                    },
                  ),
                ),
            ],
          ),
          Align(
            alignment: Alignment.bottomLeft,
            child: Padding(
              padding: const EdgeInsets.only(bottom: 12.0, left: 8.0),
              child: Row(
                children: [
                  ElevatedButton.icon(
                    onPressed: _addNewTree,
                    icon: const Icon(Icons.add),
                    label: const Text('New Tree'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: colorScheme.primaryContainer,
                      foregroundColor: colorScheme.onPrimaryContainer,
                      elevation: 4.0,
                      fixedSize: const Size(141, 56),
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(16)),
                    ),
                  ),
                  const SizedBox(width: 6),
                  ElevatedButton.icon(
                    onPressed: _showDeleteDialog,
                    icon: const Icon(Icons.delete_forever),
                    label: const Text('Delete'),
                    style: ElevatedButton.styleFrom(
                      backgroundColor: colorScheme.surface,
                      foregroundColor: colorScheme.onSurface,
                      elevation: 4.0,
                      fixedSize: const Size(141, 56),
                      shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(16)),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ],
      ),
      body: KeyBoardShortcuts(
        keysToPress: {LogicalKeyboardKey.backslash},
        onKeysPressed: () {
          if (_selectedNodes.isNotEmpty) {
            setState(() {
              for (TreeNode node in _selectedNodes) {
                TreeNode? parent = _findParent(node);
                if (parent != null) {
                  parent.children.remove(node);
                  _currentTree.detatchedNodes.add(node);
                }
              }
            });
            _saveTree(_currentTree);
          }
        },
        child: KeyBoardShortcuts(
          keysToPress: {LogicalKeyboardKey.delete},
          onKeysPressed: () {
            if (_selectedNodes.isNotEmpty) {
              setState(() {
                for (TreeNode node in _selectedNodes) {
                  if (node is! RootNode) {
                    _deleteNode(node);
                  }
                }
                _selectedNodes.clear();
              });
              _saveTree(_currentTree);
            }
          },
          child: Stack(
            children: [
              InteractiveViewer(
                minScale: 0.01,
                maxScale: 5.0,
                scaleFactor: 800,
                constrained: false,
                boundaryMargin: const EdgeInsets.all(_canvasPadding),
                transformationController: _controller,
                child: GestureDetector(
                  behavior: HitTestBehavior.opaque,
                  onTapUp: (details) {
                    Offset pos = details.localPosition;

                    TreeNode? hit = _getNodeHit(pos);

                    if (hit != null) {
                      setState(() {
                        _dragArrowPos = null;
                        _addMenuPos = null;
                        _addMenuFocusNode.unfocus();

                        _selectedNodes = [hit];
                      });
                    } else {
                      setState(() {
                        _dragArrowPos = null;
                        _addMenuPos = null;
                        _addMenuFocusNode.unfocus();

                        _selectedNodes.clear();
                      });
                    }
                  },
                  onPanStart: (details) {
                    Offset pos = details.localPosition;

                    TreeNode? hit = _getNodeHit(pos);

                    if (hit != null) {
                      if (hit.canTakeChildren) {
                        Size size = hit.key.currentContext!.size!;

                        Offset dragBoxStart = hit.position +
                            Offset(-_nodeDragAreaWidth / 2,
                                size.height - _nodeDragAreaHeight);
                        Offset dragBoxEnd = hit.position +
                            Offset(_nodeDragAreaWidth / 2, size.height);

                        if (pos >= dragBoxStart && pos <= dragBoxEnd) {
                          _dragArrowPos = pos;

                          setState(() {
                            _selectedNodes = [hit];
                          });
                          return;
                        }
                      }

                      if (!_selectedNodes.contains(hit)) {
                        setState(() {
                          _selectedNodes = [hit];
                        });
                      }
                    } else {
                      setState(() {
                        _selectionRectA = pos;
                        _selectionRectB = pos;
                      });
                    }
                  },
                  onPanUpdate: (details) {
                    if (_selectionRectA != null && _selectionRectB != null) {
                      setState(() {
                        _selectionRectB = _selectionRectB! + details.delta;
                        _selectMultiple(_selectionRectA!, _selectionRectB!);
                      });
                    } else if (_selectedNodes.isNotEmpty) {
                      Offset change = details.delta;

                      if (_dragArrowPos != null) {
                        TreeNode? hit = _getNodeHit(details.localPosition);
                        if (hit != null) {
                          setState(() {
                            _dragArrowPos = _dragArrowPos! + change;
                            _raisedNode = hit;
                          });
                        } else {
                          setState(() {
                            _dragArrowPos = _dragArrowPos! + change;
                            _raisedNode = null;
                          });
                        }
                      } else {
                        setState(() {
                          for (TreeNode node in _selectedNodes) {
                            node.position = node.position + change;
                            Size nodeSize = node.key.currentContext!.size!;
                            double halfWidth = nodeSize.width / 2;

                            if (node.position.dx - halfWidth < 0) {
                              node.position =
                                  Offset(halfWidth, node.position.dy);
                            }

                            if (node.position.dy < 0) {
                              node.position = Offset(node.position.dx, 0);
                            }

                            if (node.position.dx + halfWidth > _canvasWidth) {
                              node.position = Offset(
                                  _canvasWidth - halfWidth, node.position.dy);
                            }

                            if (node.position.dy + nodeSize.height >
                                _canvasHeight) {
                              node.position = Offset(node.position.dx,
                                  _canvasHeight - nodeSize.height);
                            }
                          }
                        });
                      }
                    }
                  },
                  onPanEnd: (details) {
                    if (_selectionRectA != null || _selectionRectB != null) {
                      setState(() {
                        _selectionRectA = null;
                        _selectionRectB = null;
                      });
                    } else if (_dragArrowPos != null) {
                      // Should only have one selected
                      TreeNode? onNode = _getNodeHit(_dragArrowPos!);

                      if (onNode != null) {
                        // Make node child
                        TreeNode? parent = _findParent(onNode);

                        setState(() {
                          if (parent != null) {
                            parent.children.remove(onNode);
                          } else {
                            // Has no parent, means it is detached
                            _currentTree.detatchedNodes.remove(onNode);
                          }
                          _selectedNodes[0].children.add(onNode);
                          _dragArrowPos = null;
                          _addMenuPos = null;
                          _addMenuFocusNode.unfocus();
                          _raisedNode = null;
                        });
                        _saveTree(_currentTree);
                      } else {
                        setState(() {
                          _addMenuPos = _dragArrowPos;
                          _addMenuFocusNode.requestFocus();
                        });
                      }
                    } else if (_selectedNodes.isNotEmpty) {
                      // Moving nodes
                      _saveTree(_currentTree);
                    }
                  },
                  child: Container(
                    width: _canvasWidth,
                    height: _canvasHeight,
                    decoration: BoxDecoration(
                      border: Border.all(width: 3, color: Colors.grey[800]!),
                      borderRadius: BorderRadius.circular(32),
                    ),
                    child: Stack(
                      children: [
                        ..._nodeWidgets(_currentTree.rootNode),
                        for (TreeNode detatched in _currentTree.detatchedNodes)
                          ..._nodeWidgets(detatched),
                        ArrowPainter(
                          painterKey: _paintKey,
                          allRoots: [
                            _currentTree.rootNode,
                            ..._currentTree.detatchedNodes
                          ],
                          selectedNodes: _selectedNodes,
                          dragArrowPos: _dragArrowPos,
                        ),
                        if (_addMenuPos != null)
                          Positioned(
                            top: _addMenuPos!.dy,
                            left: _addMenuPos!.dx,
                            child: NodeSearch(
                              focusNode: _addMenuFocusNode,
                              newNodePosition: _addMenuPos!,
                              onNodeCreated: (node) {
                                setState(() {
                                  if (node != null) {
                                    // Should only have one selected
                                    _selectedNodes[0].children.add(node);
                                    _selectedNodes = [node];
                                  }
                                  _saveTree(_currentTree);

                                  _addMenuPos = null;
                                  _dragArrowPos = null;
                                });
                              },
                            ),
                          ),
                        if (_selectionRectA != null && _selectionRectB != null)
                          Positioned(
                            top: min(_selectionRectA!.dy, _selectionRectB!.dy),
                            left: min(_selectionRectA!.dx, _selectionRectB!.dx),
                            child: DottedBorder(
                              color: Colors.grey[800]!,
                              strokeWidth: 3,
                              radius: const Radius.circular(8),
                              padding: const EdgeInsets.all(0),
                              borderType: BorderType.RRect,
                              dashPattern: const [20, 15],
                              child: SizedBox(
                                width:
                                    (_selectionRectA!.dx - _selectionRectB!.dx)
                                        .abs(),
                                height:
                                    (_selectionRectA!.dy - _selectionRectB!.dy)
                                        .abs(),
                              ),
                            ),
                          ),
                      ],
                    ),
                  ),
                ),
              ),
              Align(
                alignment: Alignment.centerRight,
                child: DetailsPanel(
                  width: 400,
                  selectedNode:
                      _selectedNodes.length == 1 ? _selectedNodes[0] : null,
                  onNodeChanged: () {
                    setState(() {});
                    _saveTree(_currentTree);
                  },
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }

  void _selectMultiple(Offset rectA, Offset rectB) {
    List<TreeNode> allRoots = [
      _currentTree.rootNode,
      ..._currentTree.detatchedNodes
    ];

    _selectedNodes.clear();
    Rect selectionRect = Rect.fromPoints(rectA, rectB);

    for (TreeNode root in allRoots) {
      for (TreeNode node in root.getAllNodes()) {
        Size nodeSize = node.key.currentContext!.size!;

        Rect nodeRect = Rect.fromLTWH(node.position.dx - (nodeSize.width / 2),
            node.position.dy, nodeSize.width, nodeSize.height);

        if (nodeRect.overlaps(selectionRect)) {
          _selectedNodes.add(node);
        }
      }
    }
  }

  void _deleteNode(TreeNode node) {
    for (TreeNode child in node.children) {
      _currentTree.detatchedNodes.add(child);
    }

    TreeNode? parent = _findParent(node);
    if (parent != null) {
      parent.children.remove(node);
    } else {
      // detached
      _currentTree.detatchedNodes.remove(node);
    }
    _saveTree(_currentTree);
  }

  TreeNode? _findParent(TreeNode child) {
    List<TreeNode> allRoots = [
      _currentTree.rootNode,
      ..._currentTree.detatchedNodes
    ];

    for (TreeNode root in allRoots) {
      for (TreeNode node in root.getAllNodes()) {
        if (node.children.contains(child)) {
          return node;
        }
      }
    }

    return null;
  }

  TreeNode? _getNodeHit(Offset mousePosition) {
    List<TreeNode> allRoots = [
      _currentTree.rootNode,
      ..._currentTree.detatchedNodes
    ];

    for (TreeNode root in allRoots) {
      for (TreeNode node in root.getAllNodes()) {
        Size size = node.key.currentContext!.size!;

        if (mousePosition >= node.position - const Offset(_nodeWidth / 2, 0) &&
            mousePosition <=
                node.position +
                    Offset(size.width - (_nodeWidth / 2), size.height)) {
          return node;
        }
      }
    }
    return null;
  }

  List<Widget> _nodeWidgets(TreeNode node) {
    bool active;
    if (node is RootNode) {
      active = node.children.isNotEmpty
          ? _activeNodes.contains(node.children[0].uuid)
          : false;
    } else {
      active = _activeNodes.contains(node.uuid);
    }

    List<Widget> nodes = [
      Positioned(
        top: node.position.dy,
        left: node.position.dx - (_nodeWidth / 2),
        child: NodeCard(
          node: node,
          active: active,
          width: _nodeWidth,
          dragAreaWidth: _nodeDragAreaWidth,
          dragAreaHeight: _nodeDragAreaHeight,
          selected: _selectedNodes.contains(node),
          raised: _raisedNode == node,
        ),
      ),
    ];

    for (TreeNode n in node.children) {
      nodes.addAll(_nodeWidgets(n));
    }

    return nodes;
  }

  void _openProjectDialog(BuildContext context) async {
    String initialDirectory = _projectDir?.path ?? fs.currentDirectory.path;
    String? projectFolder = await getDirectoryPath(
        confirmButtonText: 'Open Project', initialDirectory: initialDirectory);
    if (projectFolder != null) {
      _trees.clear();
      _treeIdx = 0;
      _selectedNodes.clear();
      _raisedNode = null;
      _dragArrowPos = null;
      _addMenuPos = null;
      _initFromProjectDir(projectFolder);
    }
  }

  void _openProject() async {
    await Future.delayed(const Duration(milliseconds: 100)); // cringe

    String? projectDir = widget.prefs.getString('currentProjectDir');
    if (projectDir != null && Platform.isMacOS) {
      if (widget.prefs.getString('macOSBookmark') != null) {
        try {
          await _bookmarks!
              .resolveBookmark(widget.prefs.getString('macOSBookmark')!);

          await _bookmarks!
              .startAccessingSecurityScopedResource(fs.file(projectDir));
        } catch (e) {
          projectDir = null;
        }
      } else {
        projectDir = null;
      }
    }

    if (mounted &&
        (projectDir == null || !fs.directory(projectDir).existsSync())) {
      projectDir = await Navigator.push(
        this.context,
        PageRouteBuilder(
          pageBuilder: (context, anim1, anim2) => const WelcomePage(),
          transitionDuration: Duration.zero,
          reverseTransitionDuration: Duration.zero,
        ),
      );
    }

    _initFromProjectDir(projectDir!);
  }

  void _initFromProjectDir(String projectDir) async {
    widget.prefs.setString('currentProjectDir', projectDir);

    if (Platform.isMacOS) {
      // Bookmark project on macos so it can be accessed again later
      String bookmark = await _bookmarks!.bookmark(fs.file(projectDir));
      widget.prefs.setString('macOSBookmark', bookmark);
    }

    // Check if WPILib project
    setState(() {
      if (fs.file(join(projectDir, 'build.gradle')).existsSync()) {
        _deployDir = fs.directory(
            join(projectDir, 'src', 'main', 'deploy', 'behavior_trees'));
      } else {
        _deployDir = fs.directory(join(projectDir, 'deploy', 'behavior_trees'));
      }
    });

    await _deployDir.create(recursive: true);

    var treeFiles = _deployDir.listSync();
    for (FileSystemEntity e in treeFiles) {
      if (basename(e.path).endsWith('.tree')) {
        String name = basenameWithoutExtension(e.path);
        String content = await fs.file(e.path).readAsString();

        _trees.add(BehaviorTree.fromJson(jsonDecode(content), name));
      }
    }

    if (_trees.isEmpty) {
      _trees.add(BehaviorTree.defaultTree());
    }

    setState(() {
      _projectDir = fs.directory(projectDir);
      _treeIdx = 0;
      _currentTree = _trees.first;
    });
  }

  void _saveTree(BehaviorTree tree) async {
    final treeFile =
        fs.file(join(_deployDir.path, '${_currentTree.name}.tree'));

    await treeFile.writeAsString(jsonEncode(_currentTree.toJson()));
  }

  void _renameTree(int treeIdx, String newName) {
    List<String> treeNames = [];
    for (BehaviorTree tree in _trees) {
      treeNames.add(tree.name);
    }

    if (treeNames.contains(newName)) {
      showDialog(
          context: this.context,
          builder: (BuildContext context) {
            return AlertDialog(
              title: const Text('Unable to Rename'),
              content: Text('The file "$newName.tree" already exists'),
              actions: [
                TextButton(
                  onPressed: Navigator.of(context).pop,
                  child: const Text('OK'),
                ),
              ],
            );
          });
    } else {
      String oldName = _trees[treeIdx].name;
      setState(() {
        _trees[treeIdx].name = newName;
      });

      final file = fs.file(join(_deployDir.path, '$oldName.tree'));

      file.exists().then((exists) {
        if (exists) {
          file.rename(join(_deployDir.path, '$newName.tree'));
        }
      });
    }
  }

  void _addNewTree() {
    List<String> treeNames = [];
    for (BehaviorTree tree in _trees) {
      treeNames.add(tree.name);
    }

    String treeName = 'New Tree';
    while (treeNames.contains(treeName)) {
      treeName = 'New $treeName';
    }

    setState(() {
      _trees.add(BehaviorTree.defaultTree(name: treeName));
    });
  }

  void _showDeleteDialog() {
    if (_trees.length > 1) {
      showDialog(
          context: this.context,
          builder: (context) {
            return AlertDialog(
              title: const Text('Delete Tree'),
              content: Text(
                  'Are you sure you want to delete the file "${_currentTree.name}.tree"?\n This cannot be undone!'),
              actions: [
                TextButton(
                  onPressed: Navigator.of(context).pop,
                  child: const Text('Cancel'),
                ),
                TextButton(
                  onPressed: () async {
                    final file = fs.file(
                        join(_deployDir.path, '${_currentTree.name}.tree'));

                    if (await file.exists()) {
                      await file.delete();
                    }

                    setState(() {
                      _trees.removeAt(_treeIdx);
                      _treeIdx = 0;
                      _currentTree = _trees[_treeIdx];
                    });

                    if (mounted) {
                      Navigator.of(context).pop();
                    }
                  },
                  child: const Text('Confirm'),
                ),
              ],
            );
          });
    }
  }
}
