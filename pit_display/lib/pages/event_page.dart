import 'dart:io';

import 'package:flutter/material.dart';
import 'package:pit_display/widgets/event/event_match_schedule.dart';
import 'package:pit_display/widgets/event/match_schedule.dart';
import 'package:pit_display/widgets/event/ranking_card.dart';
import 'package:pit_display/widgets/event/twitch_view_windows.dart';

class EventPage extends StatelessWidget {
  const EventPage({super.key});

  @override
  Widget build(BuildContext context) {
    return Row(
      mainAxisSize: MainAxisSize.max,
      children: [
        const Expanded(
          flex: 3,
          child: Column(
            children: [
              Expanded(
                flex: 35,
                child: RankingCard(),
              ),
              Expanded(
                flex: 65,
                child: MatchSchedule(),
              ),
            ],
          ),
        ),
        Expanded(
          flex: 7,
          child: Column(
            children: [
              Padding(
                padding: const EdgeInsets.all(8.0),
                child: Card(
                  clipBehavior: Clip.hardEdge,
                  child: AspectRatio(
                    aspectRatio: 16 / 9,
                    child: Platform.isWindows
                        ? const TwitchViewWindows()
                        : const Center(
                            child:
                                Text('Twitch stream only supported on windows'),
                          ),
                  ),
                ),
              ),
              const Expanded(child: EventMatchSchedule()),
            ],
          ),
        ),
      ],
    );
  }
}
