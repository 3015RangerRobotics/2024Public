package frc.robot.subsystems.shooter_joint;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;

public class ShooterJointIOPhoenix implements ShooterJointIO {
  private final TalonFX jointMotor;
  private final TalonFXSimState jointMotorSim;

  private final CANcoder jointEncoder;
  private final CANcoderSimState encoderSim;

  private final StatusSignal<Double> jointPositionSignal;
  private final StatusSignal<Double> jointVelocitySignal;
  private final StatusSignal<Double> jointAccelSignal;
  private final StatusSignal<Double> jointMotorTempSignal;
  private final StatusSignal<Double> jointMotorSupplyCurrentSignal;
  private final StatusSignal<Double> jointMotorStatorCurrentSignal;
  private final StatusSignal<Double> jointMotorVoltageSignal;
  private final StatusSignal<Double> jointMotorSupplyVoltage;
  private final StatusSignal<Double> jointMotorErrorSignal;

  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final MotionMagicVoltage positionProfiledRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final SingleJointedArmSim jointSim;

  public ShooterJointIOPhoenix() {
    jointMotor = new TalonFX(Constants.ShooterJoint.motorID, Constants.canivoreBusName);
    jointEncoder = new CANcoder(Constants.ShooterJoint.encoderID, Constants.canivoreBusName);

    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = jointEncoder.getConfigurator().refresh(encoderConfig, 2.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    } else if (refreshStatus != StatusCode.OK || encoderConfig.MagnetSensor.MagnetOffset == 0) {
      encoderConfig.MagnetSensor.MagnetOffset =
          Preferences.getDouble(
              "ShooterJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    } else {
      Preferences.setDouble("ShooterJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    }

    jointEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ShooterJoint.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    //    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.ShooterJoint.currentLimit;
    //    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.ShooterJoint.currentLimit;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = jointEncoder.getDeviceID();
    motorConfig.Feedback.RotorToSensorRatio = Constants.ShooterJoint.gearing;
    motorConfig.Slot0.kP = Constants.ShooterJoint.kP;
    motorConfig.Slot0.kI = Constants.ShooterJoint.kI;
    motorConfig.Slot0.kD = Constants.ShooterJoint.kD;
    //    motorConfig.Slot0.kV = Constants.ShooterJoint.kV;
    //    motorConfig.Slot0.kA = Constants.ShooterJoint.kA;
    motorConfig.Slot0.kS = Constants.ShooterJoint.kS;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ShooterJoint.forwardLimit.getRotations();
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ShooterJoint.reverseLimit.getRotations();
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ShooterJoint.mmMaxVel;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.ShooterJoint.mmMaxAccel;
    motorConfig.Audio.AllowMusicDurDisable = true;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    jointMotor.getConfigurator().apply(motorConfig);

    jointPositionSignal = jointEncoder.getAbsolutePosition();
    jointVelocitySignal = jointEncoder.getVelocity();
    jointAccelSignal = jointMotor.getAcceleration();
    jointMotorTempSignal = jointMotor.getDeviceTemp();
    jointMotorSupplyCurrentSignal = jointMotor.getSupplyCurrent();
    jointMotorStatorCurrentSignal = jointMotor.getStatorCurrent();
    jointMotorVoltageSignal = jointMotor.getMotorVoltage();
    jointMotorSupplyVoltage = jointMotor.getSupplyVoltage();
    jointMotorErrorSignal = jointMotor.getClosedLoopError();

    jointSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500Foc(1),
                Constants.ShooterJoint.SimInfo.MOI,
                Constants.ShooterJoint.gearing),
            DCMotor.getFalcon500Foc(1),
            Constants.ShooterJoint.gearing,
            Constants.ShooterJoint.SimInfo.armLength,
            Constants.ShooterJoint.reverseLimit.getRadians(),
            Constants.ShooterJoint.forwardLimit.getRadians(),
            false,
            0.0);

    jointMotorSim = jointMotor.getSimState();
    encoderSim = jointEncoder.getSimState();
  }

  @Override
  public void updateInputs(ShooterJointInputs inputs) {
    if (Robot.isSimulation()) {
      jointMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      encoderSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double jointVoltage = jointMotorSim.getMotorVoltage();

      jointSim.setInput(jointVoltage);
      jointSim.update(0.02);

      double jointPosRot = Units.radiansToRotations(jointSim.getAngleRads());
      double jointVelRot = Units.radiansToRotations(jointSim.getVelocityRadPerSec());

      encoderSim.setRawPosition(jointPosRot);
      encoderSim.setVelocity(jointVelRot);
    }

    double jointRot = jointPositionSignal.getValue();

    inputs.jointAngleDegrees = Units.rotationsToDegrees(jointRot);
    inputs.jointVelocityDps = Units.rotationsToDegrees(jointVelocitySignal.getValue());
    inputs.jointAccelDpsSq = Units.rotationsToDegrees(jointAccelSignal.getValue());
    inputs.jointMotorTemp = jointMotorTempSignal.getValue();
    inputs.jointMotorSupplyCurrent = jointMotorSupplyCurrentSignal.getValue();
    inputs.jointMotorStatorCurrent = jointMotorStatorCurrentSignal.getValue();
    inputs.jointMotorVoltage = jointMotorVoltageSignal.getValue();
    inputs.jointMotorSupplyVoltage = jointMotorSupplyVoltage.getValue();
    inputs.jointMotorClosedLoopError = Units.rotationsToDegrees(jointMotorErrorSignal.getValue());

    BaseStatusSignal.refreshAll(
        jointPositionSignal,
        jointVelocitySignal,
        jointAccelSignal,
        jointMotorTempSignal,
        jointMotorSupplyCurrentSignal,
        jointMotorStatorCurrentSignal,
        jointMotorVoltageSignal,
        jointMotorSupplyVoltage,
        jointMotorErrorSignal);
  }

  @Override
  public void setTargetAngle(Rotation2d angle) {
    jointMotor.setControl(positionRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void setTargetAngleProfiled(Rotation2d angle) {
    jointMotor.setControl(positionProfiledRequest.withPosition(angle.getRotations()));
  }

  @Override
  public void setVoltage(double volts) {
    jointMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void brakeMotor() {
    jointMotor.setControl(brakeRequest);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Joint Motor", jointMotor);
    subsystem.registerHardware("Joint Encoder", jointEncoder);
  }

  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, jointMotor.getPosition(), jointMotor.getVelocity(), jointMotor.getMotorVoltage());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(jointMotor);
  }
}
