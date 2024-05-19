import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:nt4/nt4.dart';

class DashboardState {
  static const String _robotAddress = kDebugMode ? '127.0.0.1' : '10.30.15.2';

  late NT4Client _client;

  late NT4Subscription _matchTimeSub;
  late NT4Subscription _redAllianceSub;
  late NT4Subscription _scoringModeSub;
  late NT4Subscription _chutePosSub;

  late NT4Topic _climbPosPub;

  int _climbPos = 1;

  bool _connected = false;

  DashboardState() {
    _client = NT4Client(
      serverBaseAddress: _robotAddress,
      onConnect: () {
        Future.delayed(const Duration(milliseconds: 200), () => _sendAll());
        _connected = true;
      },
      onDisconnect: () => _connected = false,
    );

    _matchTimeSub = _client.subscribePeriodic('/SmartDashboard/MatchTime', 1.0);
    _redAllianceSub =
        _client.subscribePeriodic('/SmartDashboard/IsRedAlliance', 1.0);
    _scoringModeSub =
        _client.subscribePeriodic('/AdvantageKit/RealOutputs/ScoringMode', 0.5);
    _chutePosSub =
        _client.subscribePeriodic('/AdvantageKit/RealOutputs/ChutePos', 0.5);

    _climbPosPub = _client.publishNewTopic(
        '/Dashboard/TargetClimbPos', NT4TypeStr.typeInt);

    _client.setProperties(_climbPosPub, false, true);

    Timer.periodic(const Duration(seconds: 1), (timer) {
      if (_connected) {
        _sendAll();
      }
    });
  }

  Stream<bool> connectionStatus() {
    return _client.connectionStatusStream().asBroadcastStream();
  }

  Stream<double> matchTime() async* {
    await for (final value in _matchTimeSub.stream()) {
      if (value is double) {
        yield value;
      }
    }
  }

  Stream<bool> isRedAlliance() async* {
    await for (final value in _redAllianceSub.stream(yieldAll: true)) {
      if (value is bool) {
        yield value;
      }
    }
  }

  Stream<int> scoringMode() async* {
    await for (final value in _scoringModeSub.stream(yieldAll: true)) {
      if (value is int) {
        yield value;
      }
    }
  }

  Stream<int> chutePos() async* {
    await for (final value in _chutePosSub.stream(yieldAll: true)) {
      if (value is int) {
        yield value;
      }
    }
  }

  void setClimbPos(int climbPos) {
    if (climbPos <= 2 && climbPos >= 0) {
      _climbPos = climbPos;
      _client.addSample(_climbPosPub, _climbPos);
    }
  }

  void _sendAll() {
    _client.addSample(_climbPosPub, _climbPos);
  }
}
