import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:pit_display/services/tba_api.dart';

class RankingCard extends StatefulWidget {
  const RankingCard({super.key});

  @override
  State<RankingCard> createState() => _RankingCardState();
}

class _RankingCardState extends State<RankingCard> {
  bool _hasData = false;

  int _rank = 0;
  double _rankingScore = 0.0;
  int _wins = 0;
  int _losses = 0;
  int _ties = 0;
  double _avgCoop = 0.0;
  double _avgMatch = 0.0;
  double _avgAuto = 0.0;
  double _avgStage = 0.0;

  late Timer _apiTimer;

  @override
  void initState() {
    super.initState();

    _getRankingInfo();
    _apiTimer = Timer.periodic(
        const Duration(seconds: 10), (timer) => _getRankingInfo());
  }

  @override
  void dispose() {
    _apiTimer.cancel();

    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    return Padding(
      padding: const EdgeInsets.all(8.0),
      child: Card(
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: !_hasData
              ? const Center(child: Text('No ranking data yet'))
              : Column(
                  crossAxisAlignment: CrossAxisAlignment.center,
                  mainAxisSize: MainAxisSize.max,
                  children: [
                    Text(
                      'Rank $_rank',
                      style: const TextStyle(fontSize: 64),
                    ),
                    const SizedBox(height: 16),
                    Row(
                      mainAxisSize: MainAxisSize.max,
                      mainAxisAlignment: MainAxisAlignment.spaceAround,
                      children: [
                        Text(
                          '${_rankingScore.toStringAsFixed(2)}RS',
                          style: const TextStyle(fontSize: 48),
                        ),
                        Text(
                          _ties > 0
                              ? '${_wins}W-${_losses}L-${_ties}T'
                              : '${_wins}W-${_losses}L',
                          style: const TextStyle(fontSize: 48),
                        ),
                      ],
                    ),
                    const SizedBox(height: 32),
                    Row(
                      mainAxisSize: MainAxisSize.max,
                      children: [
                        Expanded(
                          child: Column(
                            children: [
                              Text(
                                '${_avgCoop.toStringAsFixed(2)} Co-Op',
                                style: TextStyle(
                                  fontSize: 32,
                                  color: colorScheme.onSurfaceVariant,
                                ),
                              ),
                              Text(
                                '${_avgAuto.toStringAsFixed(2)} Auto',
                                style: TextStyle(
                                  fontSize: 32,
                                  color: colorScheme.onSurfaceVariant,
                                ),
                              ),
                            ],
                          ),
                        ),
                        const SizedBox(width: 8),
                        Expanded(
                          child: Column(
                            children: [
                              Text(
                                '${_avgMatch.toStringAsFixed(2)} Match',
                                style: TextStyle(
                                  fontSize: 32,
                                  color: colorScheme.onSurfaceVariant,
                                ),
                              ),
                              Text(
                                '${_avgStage.toStringAsFixed(2)} Stage',
                                style: TextStyle(
                                  fontSize: 32,
                                  color: colorScheme.onSurfaceVariant,
                                ),
                              ),
                            ],
                          ),
                        ),
                      ],
                    ),
                  ],
                ),
        ),
      ),
    );
  }

  void _getRankingInfo() {
    TBA.getTeamRankingData().then((responseJson) {
      if (responseJson == null) {
        setState(() {
          _hasData = false;
        });
      } else {
        setState(() {
          _hasData = true;
          _rank = responseJson['rank'];
          _rankingScore = responseJson['rankingScore'];
          _wins = responseJson['wins'];
          _losses = responseJson['losses'];
          _ties = responseJson['ties'];
          _avgMatch = responseJson['avgMatch'];
          _avgCoop = responseJson['avgCoop'];
          _avgAuto = responseJson['avgAuto'];
          _avgStage = responseJson['avgStage'];
        });
      }
    }).catchError((err) {
      if (kDebugMode) {
        print('TBA API error: $err');
      }
    });
  }
}
