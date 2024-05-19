import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ScoringMode extends StatefulWidget {
  final DashboardState dashboardState;
  final bool redAlliance;

  const ScoringMode({
    super.key,
    required this.dashboardState,
    required this.redAlliance,
  });

  @override
  State<ScoringMode> createState() => _ScoringModeState();
}

class _ScoringModeState extends State<ScoringMode> {
  int _scoringMode = 1;

  @override
  void initState() {
    super.initState();

    widget.dashboardState.scoringMode().listen((val) {
      if (val != _scoringMode) {
        setState(() {
          _scoringMode = val;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    Color activeColor = widget.redAlliance ? Colors.red[700]! : Colors.indigo;

    return SizedBox(
      width: 750,
      height: 300,
      child: Padding(
        padding: const EdgeInsets.all(8.0),
        child: Column(
          mainAxisSize: MainAxisSize.max,
          crossAxisAlignment: CrossAxisAlignment.center,
          mainAxisAlignment: MainAxisAlignment.start,
          children: [
            const Text(
              'Scoring Mode',
              style: TextStyle(fontSize: 48),
            ),
            const Divider(),
            Expanded(
              child: Row(
                children: [
                  Expanded(
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        minimumSize:
                            const Size(double.infinity, double.infinity),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        backgroundColor: _scoringMode == 0 ? activeColor : null,
                      ),
                      child: const Text(
                        'AMP',
                        style: TextStyle(fontSize: 48, color: Colors.white),
                      ),
                      onPressed: () {
                        // setState(() {
                        //   _scoringMode = 0;
                        //   widget.dashboardState.setScoringMode(_scoringMode);
                        // });
                      },
                    ),
                  ),
                  const SizedBox(width: 8),
                  Expanded(
                    child: ElevatedButton(
                      style: ElevatedButton.styleFrom(
                        minimumSize:
                            const Size(double.infinity, double.infinity),
                        shape: RoundedRectangleBorder(
                          borderRadius: BorderRadius.circular(12),
                        ),
                        backgroundColor: _scoringMode == 1 ? activeColor : null,
                      ),
                      child: const Text(
                        'Speaker',
                        style: TextStyle(fontSize: 48, color: Colors.white),
                      ),
                      onPressed: () {
                        // setState(() {
                        //   _scoringMode = 1;
                        //   widget.dashboardState.setScoringMode(_scoringMode);
                        // });
                      },
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }
}
