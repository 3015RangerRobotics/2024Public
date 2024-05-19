import 'dart:io';

import 'package:behavior_tree_editor/widgets/custom_appbar.dart';
import 'package:file_selector/file_selector.dart';
import 'package:flutter/material.dart';

class WelcomePage extends StatelessWidget {
  const WelcomePage({super.key});

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Scaffold(
      appBar: CustomAppBar(
        titleWidget: const Text('Behavior Tree Editor'),
        automaticallyImplyLeading: false,
      ),
      body: Center(
        child: ElevatedButton(
          style: ElevatedButton.styleFrom(
            backgroundColor: colorScheme.surface,
            foregroundColor: colorScheme.primary,
            elevation: 4.0,
          ),
          onPressed: () {
            _showOpenProjectDialog(context);
          },
          child: const Padding(
            padding: EdgeInsets.all(6.0),
            child: Text(
              'Open Robot Project',
              style: TextStyle(fontSize: 24),
            ),
          ),
        ),
      ),
    );
  }

  void _showOpenProjectDialog(BuildContext context) async {
    String? projectFolder = await getDirectoryPath(
      confirmButtonText: 'Open Project',
      initialDirectory: Directory.current.path,
    );

    if (!context.mounted) return;

    if (projectFolder != null) {
      Navigator.pop(context, projectFolder);
    }
  }
}
