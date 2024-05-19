import 'package:fl_chart/fl_chart.dart';
import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';

class InputVoltageGraph extends StatelessWidget {
  const InputVoltageGraph({super.key});

  @override
  Widget build(BuildContext context) {
    return Card(
      clipBehavior: Clip.hardEdge,
      child: SizedBox(
        height: 300,
        child: Stack(
          children: [
            StreamBuilder(
              stream: SystemsState.battVoltagePlot(),
              builder: (context, snapshot) {
                List<FlSpot> data = [];
                if (snapshot.hasData) {
                  for (int i = 0; i < snapshot.data!.length; i++) {
                    data.add(FlSpot(0.033 * i, snapshot.data![i]));
                  }
                }

                return LineChart(
                  LineChartData(
                      lineTouchData: LineTouchData(
                        enabled: false,
                      ),
                      titlesData: FlTitlesData(
                        topTitles: AxisTitles(
                          sideTitles: SideTitles(showTitles: false),
                        ),
                        rightTitles: AxisTitles(
                          sideTitles: SideTitles(showTitles: false),
                        ),
                        bottomTitles: AxisTitles(
                          sideTitles: SideTitles(showTitles: false),
                        ),
                        leftTitles: AxisTitles(
                          sideTitles: SideTitles(
                            showTitles: false,
                          ),
                        ),
                      ),
                      gridData: FlGridData(
                        show: true,
                        drawVerticalLine: true,
                        drawHorizontalLine: true,
                        horizontalInterval: 5,
                        verticalInterval: 2.5,
                        getDrawingHorizontalLine: (value) {
                          return FlLine(
                            color: Colors.grey.withOpacity(0.3),
                            strokeWidth: 1,
                          );
                        },
                        getDrawingVerticalLine: (value) {
                          return FlLine(
                            color: Colors.grey.withOpacity(0.3),
                            strokeWidth: 1,
                          );
                        },
                      ),
                      borderData: FlBorderData(
                        show: false,
                      ),
                      minX: 0,
                      minY: 0,
                      maxX: 10,
                      maxY: 20,
                      lineBarsData: [
                        LineChartBarData(
                          spots: data,
                          isCurved: true,
                          gradient: const LinearGradient(
                            colors: [
                              Colors.redAccent,
                              Colors.orangeAccent,
                            ],
                          ),
                          barWidth: 3,
                          isStrokeCapRound: true,
                          dotData: FlDotData(show: false),
                          belowBarData: BarAreaData(
                            show: true,
                            gradient: LinearGradient(
                              colors: [
                                Colors.redAccent.withOpacity(0.3),
                                Colors.orangeAccent.withOpacity(0.3),
                              ],
                            ),
                          ),
                        ),
                      ]),
                  swapAnimationDuration: const Duration(milliseconds: 0),
                );
              },
            ),
            const Padding(
              padding: EdgeInsets.all(8.0),
              child: Align(
                alignment: Alignment.topCenter,
                child: Text(
                  'Battery Voltage (V)',
                  style: TextStyle(fontSize: 18, fontWeight: FontWeight.bold),
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}
