import 'package:flutter/foundation.dart';
import 'package:nt4/nt4.dart';

class SystemsState {
  // static const String _robotAddress = kDebugMode ? '127.0.0.1' : '10.30.15.2';
  static const String _robotAddress = '10.30.15.2';

  static late NT4Subscription _codeRuntimeSub;
  static late NT4Subscription _battVoltageSub;
  static late NT4Subscription _canUtilSub;
  static late NT4Subscription _pdhChannelsSub;
  static final List<NT4Subscription> _motorTempSubs = [];

  static late NT4Subscription _swerveCheckRanSub;
  static late NT4Subscription _swerveStatusSub;
  static late NT4Subscription _swerveLastFaultSub;

  static late NT4Subscription _flCheckRanSub;
  static late NT4Subscription _flStatusSub;
  static late NT4Subscription _flLastFaultSub;

  static late NT4Subscription _frCheckRanSub;
  static late NT4Subscription _frStatusSub;
  static late NT4Subscription _frLastFaultSub;

  static late NT4Subscription _blCheckRanSub;
  static late NT4Subscription _blStatusSub;
  static late NT4Subscription _blLastFaultSub;

  static late NT4Subscription _brCheckRanSub;
  static late NT4Subscription _brStatusSub;
  static late NT4Subscription _brLastFaultSub;

  static late NT4Subscription _armJointCheckRanSub;
  static late NT4Subscription _armJointStatusSub;
  static late NT4Subscription _armJointLastFaultSub;

  static late NT4Subscription _armExtensionCheckRanSub;
  static late NT4Subscription _armExtensionStatusSub;
  static late NT4Subscription _armExtensionLastFaultSub;

  static late NT4Subscription _shooterJointCheckRanSub;
  static late NT4Subscription _shooterJointStatusSub;
  static late NT4Subscription _shooterJointLastFaultSub;

  static late NT4Subscription _shooterCheckRanSub;
  static late NT4Subscription _shooterStatusSub;
  static late NT4Subscription _shooterLastFaultSub;

  static late NT4Subscription _intakeCheckRanSub;
  static late NT4Subscription _intakeStatusSub;
  static late NT4Subscription _intakeLastFaultSub;

  static late NT4Subscription _uptakeCheckRanSub;
  static late NT4Subscription _uptakeStatusSub;
  static late NT4Subscription _uptakeLastFaultSub;

  static late NT4Subscription _climberCheckRanSub;
  static late NT4Subscription _climberStatusSub;
  static late NT4Subscription _climberLastFaultSub;

  static late NT4Subscription _localizationStatusSub;
  static late NT4Subscription _localizationLastFaultSub;

  static late NT4Subscription _limelightStatusSub;
  static late NT4Subscription _limelightLastFaultSub;

  static late NT4Topic _swerveCheckRunningTopic;
  static late NT4Topic _armJointCheckRunningTopic;
  static late NT4Topic _armExtensionCheckRunningTopic;
  static late NT4Topic _shooterJointCheckRunningTopic;
  static late NT4Topic _shooterCheckRunningTopic;
  static late NT4Topic _intakeCheckRunningTopic;
  static late NT4Topic _uptakeCheckRunningTopic;
  static late NT4Topic _climberCheckRunningTopic;

  static late NT4Topic _allSystemsCheckRunningTopic;

  static late NT4Client _client;

  static Stream<bool> connectionStatus() {
    return _client.connectionStatusStream().asBroadcastStream();
  }

  static Stream<List<double>> codeRuntimePlot() async* {
    List<double> values = List.generate(304, (index) => 0.0);

    await for (Object? value in _codeRuntimeSub.stream(yieldAll: true)) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<List<double>> battVoltagePlot() async* {
    List<double> values = List.generate(304, (index) => 0.0);

    await for (Object? value in _battVoltageSub.stream(yieldAll: true)) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<List<double>> canUtilPlot() async* {
    List<double> values = List.generate(608, (index) => 0.0);

    await for (Object? value in _canUtilSub.stream(yieldAll: true)) {
      if (value != null) {
        values.add(value as double);
        values.removeAt(0);
        yield values;
      }
    }
  }

  static Stream<List<double>> pdhChannels() async* {
    await for (Object? value in _pdhChannelsSub.stream()) {
      if (value != null && value is List) {
        yield [for (var x in value) x as double];
      }
    }
  }

  static Stream<double> motorTemp(int idx) async* {
    await for (Object? value in _motorTempSubs[idx].stream()) {
      if (value != null) {
        yield value as double;
      }
    }
  }

  static Stream<SystemStatus> swerveStatus() async* {
    while (true) {
      bool checkRan = (_swerveCheckRanSub.currentValue ?? false) as bool;
      String status = (_swerveStatusSub.currentValue ?? '') as String;
      String lastFault = (_swerveLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> flStatus() async* {
    while (true) {
      bool checkRan = (_flCheckRanSub.currentValue ?? false) as bool;
      String status = (_flStatusSub.currentValue ?? '') as String;
      String lastFault = (_flLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> frStatus() async* {
    while (true) {
      bool checkRan = (_frCheckRanSub.currentValue ?? false) as bool;
      String status = (_frStatusSub.currentValue ?? '') as String;
      String lastFault = (_frLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> blStatus() async* {
    while (true) {
      bool checkRan = (_blCheckRanSub.currentValue ?? false) as bool;
      String status = (_blStatusSub.currentValue ?? '') as String;
      String lastFault = (_blLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> brStatus() async* {
    while (true) {
      bool checkRan = (_brCheckRanSub.currentValue ?? false) as bool;
      String status = (_brStatusSub.currentValue ?? '') as String;
      String lastFault = (_brLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> armJointStatus() async* {
    while (true) {
      bool checkRan = (_armJointCheckRanSub.currentValue ?? false) as bool;
      String status = (_armJointStatusSub.currentValue ?? '') as String;
      String lastFault = (_armJointLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> armExtensionStatus() async* {
    while (true) {
      bool checkRan = (_armExtensionCheckRanSub.currentValue ?? false) as bool;
      String status = (_armExtensionStatusSub.currentValue ?? '') as String;
      String lastFault =
          (_armExtensionLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> shooterJointStatus() async* {
    while (true) {
      bool checkRan = (_shooterJointCheckRanSub.currentValue ?? false) as bool;
      String status = (_shooterJointStatusSub.currentValue ?? '') as String;
      String lastFault =
          (_shooterJointLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> shooterStatus() async* {
    while (true) {
      bool checkRan = (_shooterCheckRanSub.currentValue ?? false) as bool;
      String status = (_shooterStatusSub.currentValue ?? '') as String;
      String lastFault = (_shooterLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> intakeStatus() async* {
    while (true) {
      bool checkRan = (_intakeCheckRanSub.currentValue ?? false) as bool;
      String status = (_intakeStatusSub.currentValue ?? '') as String;
      String lastFault = (_intakeLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> uptakeStatus() async* {
    while (true) {
      bool checkRan = (_uptakeCheckRanSub.currentValue ?? false) as bool;
      String status = (_uptakeStatusSub.currentValue ?? '') as String;
      String lastFault = (_uptakeLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> climberStatus() async* {
    while (true) {
      bool checkRan = (_climberCheckRanSub.currentValue ?? false) as bool;
      String status = (_climberStatusSub.currentValue ?? '') as String;
      String lastFault = (_climberLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(
          checkRan: checkRan, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> localizationStatus() async* {
    while (true) {
      String status = (_localizationStatusSub.currentValue ?? '') as String;
      String lastFault =
          (_localizationLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(checkRan: true, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static Stream<SystemStatus> limelightStatus() async* {
    while (true) {
      String status = (_limelightStatusSub.currentValue ?? '') as String;
      String lastFault = (_limelightLastFaultSub.currentValue ?? '') as String;

      yield SystemStatus(checkRan: true, status: status, lastFault: lastFault);
      await Future.delayed(const Duration(milliseconds: 100));
    }
  }

  static void startSwerveCheck() async {
    _client.addSample(_swerveCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_swerveCheckRunningTopic, true);
  }

  static void startArmJointCheck() async {
    _client.addSample(_armJointCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_armJointCheckRunningTopic, true);
  }

  static void startArmExtensionCheck() async {
    _client.addSample(_armExtensionCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_armExtensionCheckRunningTopic, true);
  }

  static void startShooterJointCheck() async {
    _client.addSample(_shooterJointCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_shooterJointCheckRunningTopic, true);
  }

  static void startShooterCheck() async {
    _client.addSample(_shooterCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_shooterCheckRunningTopic, true);
  }

  static void startIntakeCheck() async {
    _client.addSample(_intakeCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_intakeCheckRunningTopic, true);
  }

  static void startUptakeCheck() async {
    _client.addSample(_uptakeCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_uptakeCheckRunningTopic, true);
  }

  static void startClimberCheck() async {
    _client.addSample(_climberCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_climberCheckRunningTopic, true);
  }

  static void startAllSystemsCheck() async {
    _client.addSample(_allSystemsCheckRunningTopic, false);
    await Future.delayed(const Duration(milliseconds: 200));
    _client.addSample(_allSystemsCheckRunningTopic, true);
  }

  static void init() {
    _client = NT4Client(
      serverBaseAddress: _robotAddress,
    );

    _codeRuntimeSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/RobotPeriodicMS', 0.033);
    _battVoltageSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/BatteryVoltage', 0.033);
    _canUtilSub =
        _client.subscribePeriodic('/AdvantageKit/RealOutputs/CANUtil', 0.033);
    _pdhChannelsSub = _client.subscribePeriodic(
        '/AdvantageKit/PowerDistribution/ChannelCurrent', 0.1);

    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/FLSwerveModule/Misc/DriveTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/FLSwerveModule/Misc/RotationTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/FRSwerveModule/Misc/DriveTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/FRSwerveModule/Misc/RotationTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/BLSwerveModule/Misc/DriveTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/BLSwerveModule/Misc/RotationTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/BRSwerveModule/Misc/DriveTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Swerve/BRSwerveModule/Misc/RotationTemperature', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/ArmJoint/JointMotorTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/ArmJoint/JointFollowerTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/ArmExtension/ArmExtensionMotorTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/ShooterJoint/JointMotorTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribePeriodic('/AdvantageKit/Shooter/TopMotorTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Shooter/BottomMotorTemp', 1.0));
    _motorTempSubs.add(
        _client.subscribePeriodic('/AdvantageKit/Intake/IntakeMotorTemp', 1.0));
    _motorTempSubs
        .add(_client.subscribePeriodic('/AdvantageKit/Uptake/MotorTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Climber/LeftClimberMotorTemp', 1.0));
    _motorTempSubs.add(_client.subscribePeriodic(
        '/AdvantageKit/Climber/RightClimberMotorTemp', 1.0));

    _swerveCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Swerve/CheckRan', 0.1);
    _swerveStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Swerve/Status', 0.1);
    _swerveLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Swerve/LastFault', 0.1);

    _flCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FLSwerveModule/CheckRan', 0.1);
    _flStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FLSwerveModule/Status', 0.1);
    _flLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FLSwerveModule/LastFault', 0.1);

    _frCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FRSwerveModule/CheckRan', 0.1);
    _frStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FRSwerveModule/Status', 0.1);
    _frLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/FRSwerveModule/LastFault', 0.1);

    _blCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BLSwerveModule/CheckRan', 0.1);
    _blStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BLSwerveModule/Status', 0.1);
    _blLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BLSwerveModule/LastFault', 0.1);

    _brCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BRSwerveModule/CheckRan', 0.1);
    _brStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BRSwerveModule/Status', 0.1);
    _brLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/BRSwerveModule/LastFault', 0.1);

    _armJointCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmJoint/CheckRan', 0.1);
    _armJointStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmJoint/Status', 0.1);
    _armJointLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmJoint/LastFault', 0.1);

    _armExtensionCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmExtension/CheckRan', 0.1);
    _armExtensionStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmExtension/Status', 0.1);
    _armExtensionLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ArmExtension/LastFault', 0.1);

    _shooterJointCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ShooterJoint/CheckRan', 0.1);
    _shooterJointStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ShooterJoint/Status', 0.1);
    _shooterJointLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/ShooterJoint/LastFault', 0.1);

    _shooterCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Shooter/CheckRan', 0.1);
    _shooterStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Shooter/Status', 0.1);
    _shooterLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Shooter/LastFault', 0.1);

    _intakeCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Intake/CheckRan', 0.1);
    _intakeStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Intake/Status', 0.1);
    _intakeLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Intake/LastFault', 0.1);

    _uptakeCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Uptake/CheckRan', 0.1);
    _uptakeStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Uptake/Status', 0.1);
    _uptakeLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Uptake/LastFault', 0.1);

    _climberCheckRanSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Climber/CheckRan', 0.1);
    _climberStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Climber/Status', 0.1);
    _climberLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Climber/LastFault', 0.1);

    _localizationStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Localization/Status', 0.1);
    _localizationLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/Localization/LastFault', 0.1);

    _limelightStatusSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/LimelightNotes/Status', 0.1);
    _limelightLastFaultSub = _client.subscribePeriodic(
        '/AdvantageKit/RealOutputs/SystemStatus/LimelightNotes/LastFault', 0.1);

    _swerveCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Swerve/SystemCheck/running',
        NT4TypeStr.typeBool);
    _armJointCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/ArmJoint/SystemCheck/running',
        NT4TypeStr.typeBool);
    _armExtensionCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/ArmExtension/SystemCheck/running',
        NT4TypeStr.typeBool);
    _shooterJointCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/ShooterJoint/SystemCheck/running',
        NT4TypeStr.typeBool);
    _shooterCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Shooter/SystemCheck/running',
        NT4TypeStr.typeBool);
    _intakeCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Intake/SystemCheck/running',
        NT4TypeStr.typeBool);
    _uptakeCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Uptake/SystemCheck/running',
        NT4TypeStr.typeBool);
    _climberCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/Climber/SystemCheck/running',
        NT4TypeStr.typeBool);
    _allSystemsCheckRunningTopic = _client.publishNewTopic(
        '/SmartDashboard/SystemStatus/AllSystemsCheck/running',
        NT4TypeStr.typeBool);
  }
}

class SystemStatus {
  final bool checkRan;
  final String status;
  final String lastFault;

  const SystemStatus({
    required this.checkRan,
    required this.status,
    required this.lastFault,
  });
}
