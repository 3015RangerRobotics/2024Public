import 'dart:collection';
import 'package:flutter/foundation.dart';

import 'package:nt4/nt4.dart';

class DebugState {
  static const String _robotAddress = kDebugMode ? '127.0.0.1' : '10.30.15.2';

  late NT4Client _client;
  late NT4Subscription _activeUUIDsSub;

  bool _isConnected = false;

  DebugState() {
    _client = NT4Client(
      serverBaseAddress: _robotAddress,
      onConnect: () => _isConnected = true,
      onDisconnect: () => _isConnected = false,
    );

    _activeUUIDsSub = _client.subscribe(
        '/AdvantageKit/RealOutputs/BehaviorTree/ActiveNodes', 0.02);
  }

  bool get isConnected => _isConnected;

  Stream<bool> connectionStatusStream() {
    return _client.connectionStatusStream();
  }

  Stream<HashSet<String>> activeNodesStream() {
    return _activeUUIDsSub.stream().map((nodes) =>
        HashSet.from((nodes as List?)?.map((e) => e as String) ?? []));
  }
}
