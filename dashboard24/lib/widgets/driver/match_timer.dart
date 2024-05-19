import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class MatchTimer extends StatelessWidget {
  final DashboardState dashboardState;

  const MatchTimer({
    super.key,
    required this.dashboardState,
  });

  @override
  Widget build(BuildContext context) {
    return StreamBuilder(
      stream: dashboardState.matchTime(),
      builder: (context, snapshot) {
        String timeStr = '0:00';

        if (snapshot.hasData && snapshot.data != -1) {
          int mins = (snapshot.data! / 60).floor();
          int secs = (snapshot.data! % 60).floor();

          timeStr = '$mins:${secs.toString().padLeft(2, '0')}';
        }

        return FittedBox(
          fit: BoxFit.fitHeight,
          child: Text(
            timeStr,
            style: const TextStyle(
              fontSize: 350,
              height: 1.0,
            ),
          ),
        );
      },
    );
  }
}
