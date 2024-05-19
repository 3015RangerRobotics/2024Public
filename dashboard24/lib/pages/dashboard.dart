import 'package:dashboard24/services/dashboard_state.dart';
import 'package:dashboard24/widgets/driver/chute_selector.dart';
import 'package:dashboard24/widgets/driver/climb_selector.dart';
import 'package:dashboard24/widgets/driver/match_timer.dart';
import 'package:dashboard24/widgets/driver/scoring_mode.dart';
import 'package:flutter/material.dart';

class Dashboard extends StatefulWidget {
  final DashboardState dashboardState;

  const Dashboard({
    super.key,
    required this.dashboardState,
  });

  @override
  State<Dashboard> createState() => _DashboardState();
}

class _DashboardState extends State<Dashboard> {
  bool _redAlliance = false;

  @override
  void initState() {
    super.initState();

    widget.dashboardState.isRedAlliance().listen((event) {
      if (event != _redAlliance) {
        setState(() {
          _redAlliance = event;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: Focus(
        skipTraversal: true,
        canRequestFocus: false,
        descendantsAreFocusable: false,
        descendantsAreTraversable: false,
        child: Stack(
          children: [
            Align(
              alignment: Alignment.bottomRight,
              child: Padding(
                padding: const EdgeInsets.all(4.0),
                child: StreamBuilder(
                  stream: widget.dashboardState.connectionStatus(),
                  builder: (context, snapshot) {
                    bool connected = snapshot.data ?? false;

                    return Text(
                      'NT4: ${connected ? 'Connected' : 'Disconnected'}',
                      style: TextStyle(
                        color: connected ? Colors.green : Colors.red,
                      ),
                    );
                  },
                ),
              ),
            ),
            Align(
              alignment: _redAlliance ? Alignment.topLeft : Alignment.topRight,
              child: ChuteSelector(
                dashboardState: widget.dashboardState,
                redAlliance: _redAlliance,
              ),
            ),
            Align(
              alignment:
                  _redAlliance ? Alignment.bottomLeft : Alignment.bottomRight,
              child: Padding(
                padding: const EdgeInsets.only(left: 40),
                child: ClimbSelector(
                  dashboardState: widget.dashboardState,
                  redAlliance: _redAlliance,
                ),
              ),
            ),
            Align(
              alignment: _redAlliance ? Alignment.topRight : Alignment.topLeft,
              child: Column(
                children: [
                  ScoringMode(
                    dashboardState: widget.dashboardState,
                    redAlliance: _redAlliance,
                  ),
                  MatchTimer(dashboardState: widget.dashboardState),
                  // PickupMode(
                  //   dashboardState: widget.dashboardState,
                  // ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
