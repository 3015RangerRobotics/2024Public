import 'package:dashboard24/services/dashboard_state.dart';
import 'package:flutter/material.dart';

class ChuteSelector extends StatefulWidget {
  final DashboardState dashboardState;
  final bool redAlliance;

  const ChuteSelector({
    super.key,
    required this.dashboardState,
    required this.redAlliance,
  });

  @override
  State<ChuteSelector> createState() => _ChuteSelectorState();
}

class _ChuteSelectorState extends State<ChuteSelector> {
  int _selected = 0;

  @override
  void initState() {
    super.initState();

    widget.dashboardState.chutePos().listen((val) {
      if (val != _selected) {
        setState(() {
          _selected = val;
        });
      }
    });
  }

  @override
  Widget build(BuildContext context) {
    Color activeColor = widget.redAlliance ? Colors.red[700]! : Colors.indigo;

    return SizedBox(
      width: 250,
      height: 250,
      child: Stack(
        children: [
          Transform.flip(
            flipX: !widget.redAlliance,
            child: Image.asset('images/chute.png'),
          ),
          Positioned(
            left: widget.redAlliance ? 205 : null,
            right: widget.redAlliance ? null : 205,
            top: 45,
            child: Transform.scale(
              scale: 3.5,
              child: Checkbox(
                value: _selected == 0,
                splashRadius: 9,
                checkColor: Colors.white,
                activeColor: activeColor,
                shape: const CircleBorder(),
                side: const BorderSide(width: 0.5, color: Colors.grey),
                onChanged: (value) {
                  // setState(() {
                  //   if (value ?? false) {
                  //     _selected = 0;
                  //     widget.dashboardState.setChutePos(_selected);
                  //   }
                  // });
                },
              ),
            ),
          ),
          Positioned(
            left: widget.redAlliance ? 125 : null,
            right: widget.redAlliance ? null : 125,
            top: 95,
            child: Transform.scale(
              scale: 3.5,
              child: Checkbox(
                value: _selected == 1,
                splashRadius: 9,
                checkColor: Colors.white,
                activeColor: activeColor,
                shape: const CircleBorder(),
                side: const BorderSide(width: 0.5, color: Colors.grey),
                onChanged: (value) {
                  // setState(() {
                  //   if (value ?? false) {
                  //     _selected = 1;
                  //     widget.dashboardState.setChutePos(_selected);
                  //   }
                  // });
                },
              ),
            ),
          ),
          Positioned(
            left: widget.redAlliance ? 45 : null,
            right: widget.redAlliance ? null : 45,
            top: 145,
            child: Transform.scale(
              scale: 3.5,
              child: Checkbox(
                value: _selected == 2,
                splashRadius: 9,
                checkColor: Colors.white,
                activeColor: activeColor,
                shape: const CircleBorder(),
                side: const BorderSide(width: 0.5, color: Colors.grey),
                onChanged: (value) {
                  // setState(() {
                  //   if (value ?? false) {
                  //     _selected = 2;
                  //     widget.dashboardState.setChutePos(_selected);
                  //   }
                  // });
                },
              ),
            ),
          ),
        ],
      ),
    );
  }
}
