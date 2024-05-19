import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:webview_windows/webview_windows.dart';

final navigatorKey = GlobalKey<NavigatorState>();

class TwitchViewWindows extends StatefulWidget {
  const TwitchViewWindows({super.key});

  @override
  State<TwitchViewWindows> createState() => _TwitchViewWindowsState();
}

class _TwitchViewWindowsState extends State<TwitchViewWindows> {
  final _controller = WebviewController();
  String _twitchChannel = '';

  @override
  void initState() {
    super.initState();

    SharedPreferences.getInstance().then((prefs) {
      _twitchChannel = prefs.getString('twitchChannel') ?? '';
      initPlatformState();
    });
  }

  @override
  void dispose() {
    _controller.dispose();
    super.dispose();
  }

  Future<void> initPlatformState() async {
    try {
      await _controller.initialize();
      await _controller.setBackgroundColor(Colors.transparent);
      await _controller.setPopupWindowPolicy(WebviewPopupWindowPolicy.deny);
      await _controller.loadUrl(
          'https://player.twitch.tv/?channel=$_twitchChannel&parent=pitdisplay.rangerrobotics.org');

      if (!mounted) {
        return;
      }
      setState(() {});
    } on PlatformException catch (e) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        showDialog(
            context: context,
            builder: (_) => AlertDialog(
                  title: const Text('Error'),
                  content: Column(
                    mainAxisSize: MainAxisSize.min,
                    crossAxisAlignment: CrossAxisAlignment.start,
                    children: [
                      Text('Code: ${e.code}'),
                      Text('Message: ${e.message}'),
                    ],
                  ),
                  actions: [
                    TextButton(
                      child: const Text('Continue'),
                      onPressed: () {
                        Navigator.of(context).pop();
                      },
                    )
                  ],
                ));
      });
    }
  }

  @override
  Widget build(BuildContext context) {
    if (!_controller.value.isInitialized) {
      return const Center(
        child: SizedBox(
          width: 64,
          height: 64,
          child: CircularProgressIndicator(),
        ),
      );
    }

    return Stack(
      children: [
        Webview(
          _controller,
          permissionRequested: _onPermissionRequested,
        ),
        StreamBuilder<LoadingState>(
          stream: _controller.loadingState,
          builder: (context, snapshot) {
            if (snapshot.hasData && snapshot.data == LoadingState.loading) {
              return const Center(
                child: SizedBox(
                  width: 64,
                  height: 64,
                  child: CircularProgressIndicator(),
                ),
              );
            } else {
              return Container();
            }
          },
        ),
      ],
    );
  }

  Future<WebviewPermissionDecision> _onPermissionRequested(
      String url, WebviewPermissionKind kind, bool isUserInitiated) async {
    final decision = await showDialog<WebviewPermissionDecision>(
      context: navigatorKey.currentContext!,
      builder: (BuildContext context) => AlertDialog(
        title: const Text('WebView permission requested'),
        content: Text('WebView has requested permission \'$kind\''),
        actions: <Widget>[
          TextButton(
            onPressed: () =>
                Navigator.pop(context, WebviewPermissionDecision.deny),
            child: const Text('Deny'),
          ),
          TextButton(
            onPressed: () =>
                Navigator.pop(context, WebviewPermissionDecision.allow),
            child: const Text('Allow'),
          ),
        ],
      ),
    );

    return decision ?? WebviewPermissionDecision.none;
  }
}
