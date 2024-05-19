import 'dart:async';
import 'dart:io';

import 'package:behavior_tree_editor/pages/editor_page.dart';
import 'package:behavior_tree_editor/services/log.dart';
import 'package:flutter/material.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:window_manager/window_manager.dart';

void main() {
  runZonedGuarded(
    () async {
      WidgetsFlutterBinding.ensureInitialized();
      await Log.init();
      await windowManager.ensureInitialized();

      FlutterError.onError = (FlutterErrorDetails details) {
        FlutterError.presentError(details);
        Log.error('Flutter Error', details.exception, details.stack);
      };

      WindowOptions windowOptions = WindowOptions(
        size: const Size(1536, 864),
        minimumSize: const Size(1280, 720),
        center: true,
        title: 'Behavior Tree Editor',
        titleBarStyle:
            Platform.isMacOS ? TitleBarStyle.normal : TitleBarStyle.hidden,
      );

      windowManager.waitUntilReadyToShow(windowOptions, () async {
        await windowManager.show();
        await windowManager.focus();
      });

      SharedPreferences prefs = await SharedPreferences.getInstance();

      runApp(BehaviorTreeEditor(
        prefs: prefs,
      ));
    },
    (error, stack) {
      Log.error('Dart error', error, stack);
      exit(1);
    },
  );
}

class BehaviorTreeEditor extends StatefulWidget {
  final SharedPreferences prefs;

  const BehaviorTreeEditor({
    super.key,
    required this.prefs,
  });

  @override
  State<BehaviorTreeEditor> createState() => _BehaviorTreeEditorState();
}

class _BehaviorTreeEditorState extends State<BehaviorTreeEditor> {
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Behavior Tree Editor',
      theme: ThemeData(
        brightness: Brightness.dark,
        colorSchemeSeed: Colors.indigo,
        useMaterial3: true,
      ),
      home: EditorPage(
        prefs: widget.prefs,
      ),
    );
  }
}
