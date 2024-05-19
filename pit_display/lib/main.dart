import 'dart:io';

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:pit_display/pages/home_page.dart';
import 'package:pit_display/services/systems_state.dart';
import 'package:pit_display/widgets/keyboard_shortcuts.dart';
import 'package:window_manager/window_manager.dart';

void main() async {
  WidgetsFlutterBinding.ensureInitialized();
  await windowManager.ensureInitialized();

  WindowOptions windowOptions = WindowOptions(
    center: true,
    title: 'Pit Display',
    size: Platform.isWindows
        ? const Size(1936, 1119)
        : const Size(1920, 1145), // Equivalent of 1920 1080 when fullscreen
  );

  windowManager.waitUntilReadyToShow(windowOptions, () async {
    await windowManager.show();
    await windowManager.focus();
  });

  SystemsState.init();

  runApp(const PitDisplay());
}

class PitDisplay extends StatelessWidget {
  const PitDisplay({super.key});

  // This widget is the root of your application.
  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      title: 'Pit Display',
      debugShowCheckedModeBanner: true,
      theme: ThemeData(
        useMaterial3: true,
        colorSchemeSeed: Colors.indigo,
        brightness: Brightness.dark,
      ),
      home: KeyBoardShortcuts(
          keysToPress: {LogicalKeyboardKey.alt, LogicalKeyboardKey.enter},
          onKeysPressed: () async {
            bool fullscreen = await windowManager.isFullScreen();
            windowManager.setFullScreen(!fullscreen);
          },
          child: const HomePage()),
    );
  }
}
