package frc.robot.subsystems.swerve_module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;

public class SwerveModuleIOPhoenix implements SwerveModuleIO {
  private final TalonFX driveMotor;
  // Odom signals
  private final StatusSignal<Double> drivePositionSignal;
  private final StatusSignal<Double> driveVelocitySignal;
  // Misc signals
  private final StatusSignal<Double> driveTempSignal;
  private final StatusSignal<Double> driveAccelSignal;
  private final StatusSignal<Double> driveSupplyCurrentSignal;
  private final StatusSignal<Double> driveStatorCurrentSignal;
  private final StatusSignal<Double> driveVoltageSignal;
  private final StatusSignal<Double> driveSupplyVoltage;
  private final StatusSignal<Double> driveErrorSignal;

  private final TalonFX rotationMotor;
  // Odom signals
  private final StatusSignal<Double> rotationPositionSignal;
  private final StatusSignal<Double> rotationVelocitySignal;
  // Misc signals
  private final StatusSignal<Double> rotationTempSignal;
  private final StatusSignal<Double> rotationAccelSignal;
  private final StatusSignal<Double> rotationSupplyCurrentSignal;
  private final StatusSignal<Double> rotationStatorCurrentSignal;
  private final StatusSignal<Double> rotationVoltageSignal;
  private final StatusSignal<Double> rotationSupplyVoltage;
  private final StatusSignal<Double> rotationErrorSignal;

  private final CANcoder rotationEncoder;

  private final BaseStatusSignal[] odometrySignals;

  private final VelocityVoltage velocitySetter = new VelocityVoltage(0.0).withEnableFOC(false);
  private final PositionVoltage rotationSetter = new PositionVoltage(0.0);
  private final VoltageOut driveVoltageRequest = new VoltageOut(0.0).withEnableFOC(false);
  private final VoltageOut rotationVoltageRequest = new VoltageOut(0.0);

  public SwerveModuleIOPhoenix(ModuleConfig config) {
    driveMotor = new TalonFX(config.driveMotorID(), Constants.canivoreBusName);
    rotationMotor = new TalonFX(config.rotationMotorID(), Constants.canivoreBusName);
    rotationEncoder = new CANcoder(config.rotationEncoderID(), Constants.canivoreBusName);

    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = rotationEncoder.getConfigurator().refresh(encoderConfig, 1.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    } else if (refreshStatus != StatusCode.OK || encoderConfig.MagnetSensor.MagnetOffset == 0) {
      encoderConfig.MagnetSensor.MagnetOffset =
          Preferences.getDouble(
              config.name() + "EncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    } else {
      Preferences.setDouble(
          config.name() + "EncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    }

    rotationEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration driveConfig = new TalonFXConfiguration();

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0.kP = Constants.SwerveModule.drivekP;
    driveConfig.Slot0.kI = Constants.SwerveModule.drivekI;
    driveConfig.Slot0.kD = Constants.SwerveModule.drivekD;
    driveConfig.Slot0.kV = Constants.SwerveModule.drivekV;
    driveConfig.Slot0.kS = Constants.SwerveModule.drivekS;
    driveConfig.CurrentLimits.StatorCurrentLimit = Constants.SwerveModule.driveCurrentLimit;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.SwerveModule.driveCurrentLimit;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.SwerveModule.driveCurrentLimit;
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    driveConfig.Audio.AllowMusicDurDisable = true;

    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.setPosition(0.0);

    TalonFXConfiguration rotationConfig = new TalonFXConfiguration();

    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.Slot0.kP = Constants.SwerveModule.rotationkP;
    rotationConfig.Slot0.kI = Constants.SwerveModule.rotationkI;
    rotationConfig.Slot0.kD = Constants.SwerveModule.rotationkD;
    //    rotationConfig.Slot0.kV = Constants.SwerveModule.rotationkV;
    //    rotationConfig.Slot0.kA = Constants.SwerveModule.rotationkA;
    rotationConfig.Slot0.kS = Constants.SwerveModule.rotationkS;
    rotationConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveModule.rotationCurrentLimit;
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotationConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rotationConfig.Feedback.FeedbackRemoteSensorID = rotationEncoder.getDeviceID();
    rotationConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    rotationConfig.Feedback.RotorToSensorRatio = Constants.SwerveModule.rotationGearing;
    rotationConfig.MotionMagic.MotionMagicAcceleration = Constants.SwerveModule.rotationMMAccel;
    rotationConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.SwerveModule.rotationMMCruiseVel;
    rotationConfig.MotionMagic.MotionMagicExpo_kV = Constants.SwerveModule.rotationkV;
    rotationConfig.MotionMagic.MotionMagicExpo_kA = Constants.SwerveModule.rotationkA;
    rotationConfig.ClosedLoopGeneral.ContinuousWrap = true;
    rotationConfig.Audio.AllowMusicDurDisable = true;

    rotationMotor.getConfigurator().apply(rotationConfig);

    drivePositionSignal = driveMotor.getPosition();
    driveVelocitySignal = driveMotor.getVelocity();

    driveTempSignal = driveMotor.getDeviceTemp();
    driveAccelSignal = driveMotor.getAcceleration();
    driveSupplyCurrentSignal = driveMotor.getSupplyCurrent();
    driveStatorCurrentSignal = driveMotor.getStatorCurrent();
    driveVoltageSignal = driveMotor.getMotorVoltage();
    driveSupplyVoltage = driveMotor.getSupplyVoltage();
    driveErrorSignal = driveMotor.getClosedLoopError();

    rotationPositionSignal = rotationEncoder.getAbsolutePosition();
    rotationVelocitySignal = rotationEncoder.getVelocity();

    rotationTempSignal = rotationMotor.getDeviceTemp();
    rotationAccelSignal = rotationMotor.getAcceleration();
    rotationSupplyCurrentSignal = rotationMotor.getSupplyCurrent();
    rotationStatorCurrentSignal = rotationMotor.getStatorCurrent();
    rotationVoltageSignal = rotationMotor.getMotorVoltage();
    rotationSupplyVoltage = rotationMotor.getSupplyVoltage();
    rotationErrorSignal = rotationMotor.getClosedLoopError();

    odometrySignals =
        new BaseStatusSignal[] {
          drivePositionSignal, driveVelocitySignal, rotationPositionSignal, rotationVelocitySignal
        };
  }

  /**
   * Update the odometry inputs. This should not refresh any signals, as that will be handled in the
   * odometry thread
   */
  @Override
  public void updateOdometryInputs(SwerveModuleOdometryInputs inputs) {
    double driveRot = drivePositionSignal.getValue();
    double angleRot = rotationPositionSignal.getValue();

    // Back out the drive rotations based on angle rotations due to coupling between the drive and
    // rotation motors
    //    driveRot -= angleRot * Constants.SwerveModule.encoderToDriveCouplingRatio;

    inputs.drivePositionRot = driveRot;
    inputs.driveVelocityRps = driveVelocitySignal.getValue();

    inputs.rotationPositionDegrees = Units.rotationsToDegrees(angleRot);
    inputs.rotationVelocityDps = Units.rotationsToDegrees(rotationVelocitySignal.getValue());
  }

  /** Update the misc inputs. This should refresh signals. */
  @Override
  public void updateMiscInputs(SwerveModuleMiscInputs inputs) {
    inputs.driveTemperature = driveTempSignal.getValue();
    inputs.driveAccelMpsSq =
        driveAccelSignal.getValue() / Constants.SwerveModule.driveRotationsPerMeter;
    inputs.driveSupplyCurrent = driveSupplyCurrentSignal.getValue();
    inputs.driveStatorCurrent = driveStatorCurrentSignal.getValue();
    inputs.driveVoltage = driveVoltageSignal.getValue();
    inputs.driveSupplyVoltage = driveSupplyVoltage.getValue();
    inputs.driveClosedLoopError =
        driveErrorSignal.getValue() / Constants.SwerveModule.driveRotationsPerMeter;

    inputs.rotationTemperature = rotationTempSignal.getValue();
    inputs.rotationAccelDpsSq = Units.rotationsToDegrees(rotationAccelSignal.getValue());
    inputs.rotationSupplyCurrent = rotationSupplyCurrentSignal.getValue();
    inputs.rotationStatorCurrent = rotationStatorCurrentSignal.getValue();
    inputs.rotationVoltage = rotationVoltageSignal.getValue();
    inputs.rotationSupplyVoltage = rotationSupplyVoltage.getValue();
    inputs.rotationClosedLoopError = Units.rotationsToDegrees(rotationErrorSignal.getValue());

    BaseStatusSignal.refreshAll(
        driveTempSignal,
        driveAccelSignal,
        driveSupplyCurrentSignal,
        driveStatorCurrentSignal,
        driveVoltageSignal,
        driveSupplyVoltage,
        driveErrorSignal,
        rotationTempSignal,
        rotationAccelSignal,
        rotationSupplyCurrentSignal,
        rotationStatorCurrentSignal,
        rotationVoltageSignal,
        rotationSupplyVoltage,
        rotationErrorSignal);
  }

  @Override
  public void setTargetDriveVelocity(double driveVelocityMps) {
    double velocity = driveVelocityMps * Constants.SwerveModule.driveRotationsPerMeter;
    driveMotor.setControl(velocitySetter.withVelocity(velocity));
  }

  @Override
  public void setTargetRotation(double rotationDegrees) {
    rotationMotor.setControl(
        rotationSetter.withPosition(Units.degreesToRotations(rotationDegrees)));
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(driveVoltageRequest.withOutput(volts));
  }

  @Override
  public void setRotationVoltage(double volts) {
    rotationMotor.setControl(rotationVoltageRequest.withOutput(volts));
  }

  public BaseStatusSignal[] getOdometrySignals() {
    return odometrySignals;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(driveMotor, rotationMotor);
  }

  @Override
  public void stopMotors() {
    driveMotor.set(0);
    rotationMotor.set(0);
  }

  @Override
  public void brakeDrive() {
    driveMotor.setControl(new StaticBrake());
  }

  @Override
  public void brakeRotation() {
    rotationMotor.setControl(new StaticBrake());
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Drive Motor", driveMotor);
    subsystem.registerHardware("Rotation Motor", rotationMotor);
    subsystem.registerHardware("Rotation Encoder", rotationEncoder);
  }
}
