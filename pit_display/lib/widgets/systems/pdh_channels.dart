import 'dart:math';

import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';

class PDHChannels extends StatelessWidget {
  const PDHChannels({super.key});

  @override
  Widget build(BuildContext context) {
    return Card(
      child: SizedBox(
        height: 300,
        child: Padding(
          padding: const EdgeInsets.all(8.0),
          child: StreamBuilder(
              stream: SystemsState.pdhChannels(),
              builder: (context, snapshot) {
                var channels = snapshot.data ?? List.generate(24, (_) => 0.0);

                return Column(
                  children: [
                    Expanded(
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          Row(
                            children: [
                              _miniUpperChannel(channels[23]),
                              _miniUpperChannel(channels[22]),
                              _miniUpperChannel(channels[21]),
                              _miniUpperChannel(channels[20]),
                            ],
                          ),
                          _upperChannel(19, channels[19]),
                          _upperChannel(18, channels[18]),
                          _upperChannel(17, channels[17]),
                          _upperChannel(16, channels[16]),
                          _upperChannel(15, channels[15]),
                          _upperChannel(14, channels[14]),
                          _upperChannel(13, channels[13]),
                          _upperChannel(12, channels[12]),
                          _upperChannel(11, channels[11]),
                          _upperChannel(10, channels[10]),
                        ],
                      ),
                    ),
                    const Divider(thickness: 0.5),
                    Expanded(
                      child: Row(
                        mainAxisAlignment: MainAxisAlignment.spaceBetween,
                        children: [
                          const SizedBox(width: 80),
                          _lowerChannel(0, channels[0]),
                          _lowerChannel(1, channels[1]),
                          _lowerChannel(2, channels[2]),
                          _lowerChannel(3, channels[3]),
                          _lowerChannel(4, channels[4]),
                          _lowerChannel(5, channels[5]),
                          _lowerChannel(6, channels[6]),
                          _lowerChannel(7, channels[7]),
                          _lowerChannel(8, channels[8]),
                          _lowerChannel(9, channels[9]),
                        ],
                      ),
                    ),
                  ],
                );
              }),
        ),
      ),
    );
  }

  Widget _miniUpperChannel(double current) {
    int currentDraw = current.round();

    return Column(
      mainAxisAlignment: MainAxisAlignment.end,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        const Expanded(child: SizedBox(width: 20)),
        Container(
          width: 10,
          height: min(max(1, 80 * (currentDraw / 10)), 80),
          color: Colors.green,
        ),
        const SizedBox(height: 8),
        Text('${currentDraw}A', style: const TextStyle(fontSize: 12)),
      ],
    );
  }

  Widget _upperChannel(int channel, double current) {
    int currentDraw = current.round();

    return Column(
      mainAxisAlignment: MainAxisAlignment.end,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        Text('Ch. $channel'),
        const Expanded(child: SizedBox(width: 50)),
        Container(
          width: 20,
          height: min(max(1, 80 * (currentDraw / 60)), 80),
          color: Colors.green,
        ),
        const SizedBox(height: 8),
        Text('${currentDraw}A'),
      ],
    );
  }

  Widget _lowerChannel(int channel, double current) {
    int currentDraw = current.round();

    return Column(
      mainAxisAlignment: MainAxisAlignment.start,
      crossAxisAlignment: CrossAxisAlignment.center,
      children: [
        Text('${currentDraw}A'),
        const SizedBox(height: 8),
        Container(
          width: 20,
          height: min(max(1, 80 * (currentDraw / 60)), 80),
          color: Colors.green,
        ),
        const Expanded(child: SizedBox(width: 50)),
        Text('Ch. $channel'),
      ],
    );
  }
}
