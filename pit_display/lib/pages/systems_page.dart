import 'package:flutter/material.dart';
import 'package:pit_display/services/systems_state.dart';
import 'package:pit_display/widgets/systems/can_util_graph.dart';
import 'package:pit_display/widgets/systems/code_performance_graph.dart';
import 'package:pit_display/widgets/systems/input_voltage_graph.dart';
import 'package:pit_display/widgets/systems/motor_temps.dart';
import 'package:pit_display/widgets/systems/pdh_channels.dart';

class SystemsPage extends StatefulWidget {
  const SystemsPage({super.key});

  @override
  State<SystemsPage> createState() => _SystemsPageState();
}

class _SystemsPageState extends State<SystemsPage> {
  final List<bool> _items = List.generate(29, (index) => false);

  @override
  Widget build(BuildContext context) {
    return Stack(
      children: [
        Row(
          children: [
            Expanded(
              flex: 4,
              child: Padding(
                padding: const EdgeInsets.fromLTRB(8.0, 8.0, 0.0, 8.0),
                child: Card(
                  child: ListView(
                    children: [
                      const Text(
                        'Checklist',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 42),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Bumpers + Covers Off, Pins In', 0),
                      const SizedBox(height: 8),
                      const Text(
                        'Mechanical',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Swerve Inspection', 1),
                      const SizedBox(height: 8),
                      const Text(
                        'Climber',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 24),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Lead Screw', 2),
                      _checklistItem('Falcon', 3),
                      _checklistItem('Bearing Block', 4),
                      _checklistItem('Death Screw', 5),
                      const SizedBox(height: 8),
                      Row(
                        mainAxisSize: MainAxisSize.max,
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Expanded(
                            child: Column(
                              children: [
                                const Text(
                                  'Uptake',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(fontSize: 24),
                                ),
                                const Divider(
                                  thickness: 0.5,
                                  indent: 8,
                                  endIndent: 8,
                                ),
                                _checklistItem(
                                    'Hotdog Rollers', 6, Colors.redAccent),
                                _checklistItem(
                                    'Belts & Pulleys', 7, Colors.redAccent),
                              ],
                            ),
                          ),
                          Expanded(
                            child: Column(
                              children: [
                                const Text(
                                  'A-Frame',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(fontSize: 24),
                                ),
                                const Divider(
                                  thickness: 0.5,
                                  indent: 8,
                                  endIndent: 8,
                                ),
                                _checklistItem(
                                    'Turnbuckle', 8, Colors.indigoAccent),
                                _checklistItem(
                                    'Rotate Arm', 9, Colors.indigoAccent),
                                _checklistItem(
                                    'Gearbox', 10, Colors.indigoAccent),
                              ],
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 8),
                      const Text(
                        'Arm',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 24),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Walk the Arm', 11),
                      _checklistItem('Death Screw', 12),
                      const SizedBox(height: 8),
                      Row(
                        mainAxisSize: MainAxisSize.max,
                        crossAxisAlignment: CrossAxisAlignment.start,
                        children: [
                          Expanded(
                            child: Column(
                              children: [
                                const Text(
                                  'Shooter',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(fontSize: 24),
                                ),
                                const Divider(
                                  thickness: 0.5,
                                  indent: 8,
                                  endIndent: 8,
                                ),
                                _checklistItem(
                                    'Move Wrist', 13, Colors.redAccent),
                                _checklistItem('Spin Shooter Wheels', 14,
                                    Colors.redAccent),
                                _checklistItem(
                                    'Belts & Pulleys', 15, Colors.redAccent),
                              ],
                            ),
                          ),
                          Expanded(
                            child: Column(
                              children: [
                                const Text(
                                  'Intake',
                                  textAlign: TextAlign.center,
                                  style: TextStyle(fontSize: 24),
                                ),
                                const Divider(
                                  thickness: 0.5,
                                  indent: 8,
                                  endIndent: 8,
                                ),
                                _checklistItem(
                                    'Pulleys', 16, Colors.indigoAccent),
                                _checklistItem(
                                    'Motor Happy', 17, Colors.indigoAccent),
                                _checklistItem(
                                    'Shaft Retention', 18, Colors.indigoAccent),
                                _checklistItem(
                                    'Not Crunchy', 19, Colors.indigoAccent),
                              ],
                            ),
                          ),
                        ],
                      ),
                      const SizedBox(height: 8),
                      const Text(
                        'Electrical',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Battery Connections', 19),
                      _checklistItem('Camera Cables', 20),
                      _checklistItem('Wire Inspection', 21),
                      _checklistItem('Limelight', 22),
                      _checklistItem('Orange Pis', 23),
                      _checklistItem('RoboRIO + Ethernet Connections', 24),
                      _checklistItem('CANcoders', 24),
                      _checklistItem('Status Lights', 25),
                      const SizedBox(height: 8),
                      const Text(
                        'After Checks',
                        textAlign: TextAlign.center,
                        style: TextStyle(fontSize: 32),
                      ),
                      const Divider(
                        thickness: 0.5,
                        indent: 8,
                        endIndent: 8,
                      ),
                      _checklistItem('Bumpers + Covers On', 26),
                      _checklistItem('Robot On', 27),
                      _checklistItem('Systems Check', 28),
                    ],
                  ),
                ),
              ),
            ),
            Expanded(
              flex: 7,
              child: Column(
                children: [
                  Padding(
                    padding: const EdgeInsets.fromLTRB(0.0, 8.0, 8.0, 8.0),
                    child: Row(
                      // mainAxisSize: MainAxisSize.max,
                      crossAxisAlignment: CrossAxisAlignment.start,
                      children: [
                        const Expanded(
                          child: Column(
                            children: [
                              MotorTemps(),
                              PDHChannels(),
                              Row(
                                children: [
                                  Expanded(child: CodePerformanceGraph()),
                                  Expanded(child: InputVoltageGraph()),
                                ],
                              ),
                              Row(
                                children: [
                                  Expanded(child: CANUtilGraph()),
                                ],
                              ),
                            ],
                          ),
                        ),
                        Column(
                          children: [
                            _swerveStatusCard(),
                            _armJointStatusCard(),
                            _armExtensionStatusCard(),
                            _shooterJointStatusCard(),
                            _shooterStatusCard(),
                            _intakeStatusCard(),
                            _uptakeStatusCard(),
                            _climberStatusCard(),
                            _localizationStatusCard(),
                            _limelightStatusCard(),
                            const SizedBox(height: 4),
                            const SizedBox(
                              width: 500,
                              child: FloatingActionButton.extended(
                                onPressed: SystemsState.startAllSystemsCheck,
                                label: Text('Run All Checks'),
                                icon: Icon(Icons.check),
                              ),
                            ),
                            StreamBuilder(
                              stream: SystemsState.connectionStatus(),
                              builder: (context, snapshot) {
                                bool connected = snapshot.data ?? false;

                                if (connected) {
                                  return const Text(
                                    'Robot Status: Connected',
                                    style: TextStyle(color: Colors.green),
                                  );
                                } else {
                                  return const Text(
                                    'Robot Status: Disconnected',
                                    style: TextStyle(color: Colors.red),
                                  );
                                }
                              },
                            ),
                          ],
                        ),
                      ],
                    ),
                  ),
                ],
              ),
            ),
          ],
        ),
      ],
    );
  }

  Widget _checklistItem(String title, int itemIndex, [Color? textColor]) {
    return CheckboxListTile(
      value: _items[itemIndex],
      controlAffinity: ListTileControlAffinity.leading,
      title: Text(
        title,
        style: TextStyle(fontSize: 20, color: textColor),
      ),
      visualDensity: VisualDensity.compact,
      onChanged: (value) {
        setState(() {
          _items[itemIndex] = value ?? false;
        });
      },
    );
  }

  Widget _armJointStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.armJointStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Arm Joint',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startArmJointCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _armExtensionStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.armExtensionStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Arm Extension',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startArmExtensionCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _shooterJointStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.shooterJointStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Wrist',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startShooterJointCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _shooterStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.shooterStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Shooter',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startShooterCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
          ],
        ),
      ),
    );
  }

  Widget _swerveStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
              stream: SystemsState.swerveStatus(),
              builder: (context, snapshot) {
                SystemStatus status = snapshot.data ??
                    const SystemStatus(
                        checkRan: false, status: '', lastFault: '');

                Color statusColor = Colors.grey;
                IconData statusIcon = Icons.question_mark_rounded;
                if (status.checkRan) {
                  if (status.status == 'OK') {
                    statusColor = Colors.green;
                    statusIcon = Icons.check_circle_outline_rounded;
                  } else if (status.status == 'WARNING') {
                    statusColor = Colors.yellow;
                    statusIcon = Icons.warning_amber_rounded;
                  } else if (status.status == 'ERROR') {
                    statusColor = Colors.red;
                    statusIcon = Icons.error_outline_rounded;
                  }
                }

                return ListTile(
                  title: const Text(
                    'Swerve',
                    style: TextStyle(fontSize: 20),
                  ),
                  leading: Icon(
                    statusIcon,
                    color: statusColor,
                  ),
                  subtitle: status.lastFault.isNotEmpty
                      ? Text(status.lastFault)
                      : null,
                  trailing: const ElevatedButton(
                    onPressed: SystemsState.startSwerveCheck,
                    child: Text('Run Check'),
                  ),
                );
              },
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.flStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('FL Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.frStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('FR Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.blStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('BL Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
            Padding(
              padding: const EdgeInsets.only(left: 32.0),
              child: StreamBuilder(
                stream: SystemsState.brStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text('BR Module'),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                  );
                },
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget _intakeStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.intakeStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Intake',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    trailing: const ElevatedButton(
                      onPressed: SystemsState.startIntakeCheck,
                      child: Text('Run Check'),
                    ),
                  );
                }),
          ],
        ),
      ),
    );
  }

  Widget _uptakeStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.uptakeStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Uptake',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    trailing: const ElevatedButton(
                      onPressed: SystemsState.startUptakeCheck,
                      child: Text('Run Check'),
                    ),
                  );
                }),
          ],
        ),
      ),
    );
  }

  Widget _climberStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.climberStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Climber',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                    trailing: const ElevatedButton(
                      onPressed: SystemsState.startClimberCheck,
                      child: Text('Run Check'),
                    ),
                  );
                }),
          ],
        ),
      ),
    );
  }

  Widget _localizationStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.localizationStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Localization',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                  );
                }),
          ],
        ),
      ),
    );
  }

  Widget _limelightStatusCard() {
    return Card(
      child: SizedBox(
        width: 500,
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            StreamBuilder(
                stream: SystemsState.limelightStatus(),
                builder: (context, snapshot) {
                  SystemStatus status = snapshot.data ??
                      const SystemStatus(
                          checkRan: false, status: '', lastFault: '');

                  Color statusColor = Colors.grey;
                  IconData statusIcon = Icons.question_mark_rounded;
                  if (status.checkRan) {
                    if (status.status == 'OK') {
                      statusColor = Colors.green;
                      statusIcon = Icons.check_circle_outline_rounded;
                    } else if (status.status == 'WARNING') {
                      statusColor = Colors.yellow;
                      statusIcon = Icons.warning_amber_rounded;
                    } else if (status.status == 'ERROR') {
                      statusColor = Colors.red;
                      statusIcon = Icons.error_outline_rounded;
                    }
                  }

                  return ListTile(
                    title: const Text(
                      'Limelight',
                      style: TextStyle(fontSize: 20),
                    ),
                    leading: Icon(
                      statusIcon,
                      color: statusColor,
                    ),
                    subtitle: status.lastFault.isNotEmpty
                        ? Text(status.lastFault)
                        : null,
                  );
                }),
          ],
        ),
      ),
    );
  }
}
