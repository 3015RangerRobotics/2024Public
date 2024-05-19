import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:pit_display/model/match.dart';
import 'package:pit_display/services/tba_api.dart';
import 'package:shared_preferences/shared_preferences.dart';

class EventMatchSchedule extends StatefulWidget {
  const EventMatchSchedule({super.key});

  @override
  State<EventMatchSchedule> createState() => _EventMatchScheduleState();
}

class _EventMatchScheduleState extends State<EventMatchSchedule> {
  List<Match> _matches = [];

  late Timer _apiTimer;
  SharedPreferences? _prefs;

  @override
  void initState() {
    super.initState();

    SharedPreferences.getInstance().then((prefs) {
      _prefs = prefs;
    });

    _getMatchSchedule();
    _apiTimer = Timer.periodic(
        const Duration(seconds: 10), (timer) => _getMatchSchedule());
  }

  @override
  void dispose() {
    _apiTimer.cancel();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Column(
        children: [
          for (Match match in _matches)
            (match is FinishedMatch)
                ? _finshedMatchTile(match, context)
                : _upcomingMatchTile(match as UpcomingMatch, context),
        ],
      ),
    );
  }

  void _getMatchSchedule() {
    TBA.getShortEventMatchSchedule().then((matches) {
      if (matches == null) {
        setState(() {
          _matches = [];
        });
      } else {
        setState(() {
          _matches = matches;
        });
      }
    }).catchError((err) {
      if (kDebugMode) {
        print('TBA API err: $err');
      }
    });
  }

  Widget _upcomingMatchTile(UpcomingMatch match, BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    int etaMinutes =
        (match.estimatedStartTime.difference(DateTime.now()).inSeconds / 60)
            .round();

    return Card(
      child: SizedBox(
        height: 74,
        child: ListTile(
          leading:
              Text(match.matchNumber, style: const TextStyle(fontSize: 28)),
          title: Padding(
            padding: const EdgeInsets.only(bottom: 8.0),
            child: Row(
              children: [
                const SizedBox(width: 128),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[0], colorScheme.error),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[1], colorScheme.error),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[2], colorScheme.error),
                  ),
                ),
                const SizedBox(width: 64),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[0], colorScheme.primary),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[1], colorScheme.primary),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[2], colorScheme.primary),
                  ),
                ),
              ],
            ),
          ),
          trailing: Padding(
            padding: const EdgeInsets.only(bottom: 8.0),
            child: SizedBox(
              width: 250,
              child: Center(
                child: etaMinutes <= 3
                    ? const Text('<3m', style: TextStyle(fontSize: 28))
                    : Text(
                        etaMinutes < 60
                            ? '~${etaMinutes}m'
                            : '~${(etaMinutes / 60).floor()}:${(etaMinutes % 60).toString().padLeft(2, '0')}',
                        style: const TextStyle(fontSize: 28),
                      ),
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _finshedMatchTile(FinishedMatch match, BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    Color? matchColor = colorScheme.surfaceVariant;
    if (match.outcome == Outcome.redWin) {
      matchColor = colorScheme.errorContainer;
    } else if (match.outcome == Outcome.blueWin) {
      matchColor = colorScheme.primaryContainer;
    }

    return Card(
      color: matchColor,
      child: SizedBox(
        height: 74,
        child: ListTile(
          leading:
              Text(match.matchNumber, style: const TextStyle(fontSize: 28)),
          title: Padding(
            padding: const EdgeInsets.only(bottom: 8.0),
            child: Row(
              children: [
                const SizedBox(width: 128),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[0], colorScheme.error),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[1], colorScheme.error),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.redTeams[2], colorScheme.error),
                  ),
                ),
                const SizedBox(width: 64),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[0], colorScheme.primary),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[1], colorScheme.primary),
                  ),
                ),
                SizedBox(
                  width: 104,
                  child: Center(
                    child: _teamNumber(match.blueTeams[2], colorScheme.primary),
                  ),
                ),
              ],
            ),
          ),
          trailing: SizedBox(
            width: 250,
            child: Padding(
              padding: const EdgeInsets.only(bottom: 8.0),
              child: Row(
                children: [
                  SizedBox(
                    width: 104,
                    child: Center(
                      child: Text(
                        match.redScore.toString(),
                        style:
                            TextStyle(color: colorScheme.error, fontSize: 28),
                      ),
                    ),
                  ),
                  const Text('-', style: TextStyle(fontSize: 28)),
                  SizedBox(
                    width: 104,
                    child: Center(
                      child: Text(
                        match.blueScore.toString(),
                        style:
                            TextStyle(color: colorScheme.primary, fontSize: 28),
                      ),
                    ),
                  ),
                ],
              ),
            ),
          ),
        ),
      ),
    );
  }

  Widget _teamNumber(int team, Color color) {
    return Text(
      team.toString(),
      style: TextStyle(
        color: color,
        fontSize: 28,
        decoration: (team == _prefs?.getInt('teamNumber'))
            ? TextDecoration.underline
            : null,
      ),
    );
  }
}
