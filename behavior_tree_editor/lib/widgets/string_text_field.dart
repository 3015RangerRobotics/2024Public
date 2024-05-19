import 'package:flutter/material.dart';

class StringTextField extends StatelessWidget {
  final String initialText;
  final String label;
  final double height;
  final bool enabled;
  final ValueChanged<String>? onSubmitted;

  late final TextEditingController _controller;

  StringTextField({
    super.key,
    required this.initialText,
    required this.label,
    this.height = 42,
    this.onSubmitted,
    this.enabled = true,
  }) {
    _controller = _getController(initialText);
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return SizedBox(
      height: height,
      child: Focus(
        skipTraversal: true,
        onFocusChange: (hasFocus) {
          if (!hasFocus) {
            _onSubmitted(_controller.text);
          }
        },
        child: TextField(
          enabled: enabled,
          controller: _controller,
          style: TextStyle(fontSize: 14, color: colorScheme.onSurface),
          decoration: InputDecoration(
            contentPadding: const EdgeInsets.fromLTRB(8, 4, 8, 4),
            labelText: label,
            border: OutlineInputBorder(borderRadius: BorderRadius.circular(8)),
          ),
        ),
      ),
    );
  }

  void _onSubmitted(String val) {
    onSubmitted?.call(val);
  }

  TextEditingController _getController(String text) {
    return TextEditingController(text: text)
      ..selection =
          TextSelection.fromPosition(TextPosition(offset: text.length));
  }
}
