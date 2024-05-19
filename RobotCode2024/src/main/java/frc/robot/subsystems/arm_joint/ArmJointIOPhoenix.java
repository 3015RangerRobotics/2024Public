package frc.robot.subsystems.arm_joint;

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

public class ArmJointIOPhoenix implements ArmJointIO {
  private final TalonFX jointMotor;
  private final TalonFXSimState jointMotorSim;

  //  private final TalonFX jointMotorFollower;

  private final CANcoder jointEncoder;
  private final CANcoderSimState encoderSim;

  private final StatusSignal<Double> jointPositiionSignal;
  private final StatusSignal<Double> jointVelocitySignal;
  private final StatusSignal<Double> jointAccelSignal;
  private final StatusSignal<Double> jointErrorSignal;
  private final StatusSignal<Double> jointMotorTempSignal;
  private final StatusSignal<Double> jointMotorSupplyCurrentSignal;
  private final StatusSignal<Double> jointMotorStatorCurrentSignal;

  private final StatusSignal<Double> jointMotorSupplyVoltage;
  private final StatusSignal<Double> jointMotorVoltageSignal;
  //  private final StatusSignal<Double> jointFollowerTempSignal;
  //  private final StatusSignal<Double> jointFollowerSupplyCurrentSignal;
  //  private final StatusSignal<Double> jointFollowerStatorCurrentSignal;
  //  private final StatusSignal<Double> jointFollowerVoltageSignal;

  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(
          0.0,
          Constants.ArmJoint.mmMaxVel,
          Constants.ArmJoint.mmMaxAccel,
          Constants.ArmJoint.mmMaxJerk);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final SingleJointedArmSim jointSim;

  public ArmJointIOPhoenix() {
    jointMotor = new TalonFX(Constants.ArmJoint.motorID, Constants.canivoreBusName);
    jointEncoder = new CANcoder(Constants.ArmJoint.encoderID, Constants.canivoreBusName);
    //    jointMotorFollower = new TalonFX(Constants.ArmJoint.followerMotorID,
    // Constants.canivoreBusName);

    // Load the config from the encoder so we don't overwrite the offset
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    StatusCode refreshStatus = jointEncoder.getConfigurator().refresh(encoderConfig, 2.0);

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    // Phoenix has a hardcoded offset in sim
    if (Robot.isSimulation()) {
      encoderConfig.MagnetSensor.MagnetOffset = 0.25;
    } else if (refreshStatus != StatusCode.OK || encoderConfig.MagnetSensor.MagnetOffset == 0) {
      encoderConfig.MagnetSensor.MagnetOffset =
          Preferences.getDouble("ArmJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    } else {
      Preferences.setDouble("ArmJointEncoderOffset", encoderConfig.MagnetSensor.MagnetOffset);
    }

    jointEncoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ArmJoint.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    motorConfig.Feedback.FeedbackRemoteSensorID = jointEncoder.getDeviceID();
    motorConfig.Feedback.RotorToSensorRatio = Constants.ArmJoint.gearing;
    motorConfig.Slot0.kP = Constants.ArmJoint.kP;
    motorConfig.Slot0.kI = Constants.ArmJoint.kI;
    motorConfig.Slot0.kD = Constants.ArmJoint.kD;
    //    motorConfig.Slot0.kV = Constants.ArmJoint.kV;
    //    motorConfig.Slot0.kA = Constants.ArmJoint.kA;
    motorConfig.Slot0.kS = Constants.ArmJoint.kS;
    motorConfig.Slot0.kG = Constants.ArmJoint.kG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ArmJoint.forwardLimit.getRotations();
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ArmJoint.reverseLimit.getRotations();
    motorConfig.MotionMagic.MotionMagicExpo_kV = Constants.ArmJoint.kV;
    motorConfig.MotionMagic.MotionMagicExpo_kA = Constants.ArmJoint.kA;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmJoint.mmMaxVel;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmJoint.mmMaxAccel;
    motorConfig.MotionMagic.MotionMagicJerk = Constants.ArmJoint.mmMaxJerk;
    motorConfig.Audio.AllowMusicDurDisable = true;
    motorConfig.ClosedLoopGeneral.ContinuousWrap = true;

    jointMotor.getConfigurator().apply(motorConfig);
    //    jointMotorFollower.getConfigurator().apply(motorConfig);

    jointPositiionSignal = jointEncoder.getAbsolutePosition();
    jointVelocitySignal = jointEncoder.getVelocity();
    jointAccelSignal = jointMotor.getAcceleration();
    jointErrorSignal = jointMotor.getClosedLoopError();
    jointMotorTempSignal = jointMotor.getDeviceTemp();
    jointMotorSupplyCurrentSignal = jointMotor.getSupplyCurrent();
    jointMotorStatorCurrentSignal = jointMotor.getStatorCurrent();
    jointMotorVoltageSignal = jointMotor.getMotorVoltage();
    jointMotorSupplyVoltage = jointMotor.getSupplyVoltage();
    //    jointFollowerTempSignal = jointMotorFollower.getDeviceTemp();
    //    jointFollowerSupplyCurrentSignal = jointMotorFollower.getSupplyCurrent();
    //    jointFollowerStatorCurrentSignal = jointMotorFollower.getStatorCurrent();
    //    jointFollowerVoltageSignal = jointMotorFollower.getMotorVoltage();

    jointSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500Foc(2),
                Constants.ArmJoint.SimInfo.MOI,
                Constants.ArmJoint.gearing),
            DCMotor.getFalcon500Foc(2),
            Constants.ArmJoint.gearing,
            Constants.ArmJoint.SimInfo.armLength,
            Constants.ArmJoint.reverseLimit.getRadians(),
            Constants.ArmJoint.forwardLimit.getRadians(),
            true,
            Constants.ArmJoint.reverseLimit.getRadians());

    jointMotorSim = jointMotor.getSimState();
    encoderSim = jointEncoder.getSimState();

    //    jointMotorFollower.setControl(new Follower(jointMotor.getDeviceID(), false));
  }

  /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmJointInputs inputs) {
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

    double jointRot = jointPositiionSignal.getValue();

    inputs.jointAngleDegrees = Units.rotationsToDegrees(jointRot);
    inputs.jointVelocityDps = Units.rotationsToDegrees(jointVelocitySignal.getValue());
    inputs.jointAccelDpsSq = Units.rotationsToDegrees(jointAccelSignal.getValue());
    inputs.jointClosedLoopError = Units.rotationsToDegrees(jointErrorSignal.getValue());
    inputs.jointMotorTemp = jointMotorTempSignal.getValue();
    inputs.jointMotorSupplyCurrent = jointMotorSupplyCurrentSignal.getValue();
    inputs.jointMotorStatorCurrent = jointMotorStatorCurrentSignal.getValue();
    inputs.jointMotorVoltage = jointMotorVoltageSignal.getValue();
    inputs.jointMotorSupplyVoltage = jointMotorSupplyVoltage.getValue();
    //    inputs.jointFollowerTemp = jointFollowerTempSignal.getValue();
    //    inputs.jointFollowerSupplyCurrent = jointFollowerSupplyCurrentSignal.getValue();
    //    inputs.jointFollowerStatorCurrent = jointFollowerStatorCurrentSignal.getValue();
    //    inputs.jointFollowerVoltage = jointFollowerVoltageSignal.getValue();

    BaseStatusSignal.refreshAll(
        jointPositiionSignal,
        jointVelocitySignal,
        jointAccelSignal,
        jointErrorSignal,
        jointMotorTempSignal,
        jointMotorSupplyCurrentSignal,
        jointMotorStatorCurrentSignal,
        jointMotorVoltageSignal,
        jointMotorSupplyVoltage);
  }

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle) {
    jointMotor.setControl(
        positionRequest
            .withPosition(angle.getRotations())
            .withVelocity(Constants.ArmJoint.mmMaxVel)
            .withAcceleration(Constants.ArmJoint.mmMaxAccel));
  }

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    jointMotor.setControl(
        positionRequest
            .withPosition(angle.getRotations())
            .withVelocity(maxVel)
            .withAcceleration(maxAccel));
  }

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {
    jointMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void brakeMotors() {
    jointMotor.setControl(brakeRequest);
  }

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Joint Motor", jointMotor);
    //    subsystem.registerHardware("Joint Follower", jointMotorFollower);
    subsystem.registerHardware("Joint Encoder", jointEncoder);
  }

  /** Optimize status signals for running sysID */
  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        jointEncoder.getAbsolutePosition(),
        jointEncoder.getVelocity(),
        jointMotor.getMotorVoltage());
  }

  /**
   * Get a list of all devices to be used for orchestra commands
   *
   * @return Orchestra compatible CTRE devices
   */
  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(jointMotor);
  }
}
