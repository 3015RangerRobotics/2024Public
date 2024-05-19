package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;

public class ClimberIOPhoenix implements ClimberIO {
  private final TalonFX leftMotor;
  private final TalonFXSimState leftMotorSim;

  private final TalonFX rightMotor;
  private final TalonFXSimState rightMotorSim;

  private final StatusSignal<Double> leftClimberPositionSignal;
  private final StatusSignal<Double> leftClimberVelocitySignal;
  private final StatusSignal<Double> leftClimberAccelSignal;
  private final StatusSignal<Double> leftClimberMotorTempSignal;
  private final StatusSignal<Double> leftClimberSupplyCurrentSignal;
  private final StatusSignal<Double> leftClimberStatorCurrentSignal;
  private final StatusSignal<Double> leftClimberVoltageSignal;

  private final StatusSignal<Double> leftClimberSupplyVoltage;
  private final StatusSignal<Double> leftClimberErrorSignal;

  private final StatusSignal<Double> rightClimberPositionSignal;
  private final StatusSignal<Double> rightClimberVelocitySignal;
  private final StatusSignal<Double> rightClimberAccelSignal;
  private final StatusSignal<Double> rightClimberMotorTempSignal;
  private final StatusSignal<Double> rightClimberSupplyCurrentSignal;
  private final StatusSignal<Double> rightClimberStatorCurrentSignal;
  private final StatusSignal<Double> rightClimberVoltageSignal;
  private final StatusSignal<Double> rightClimberSupplyVoltage;
  private final StatusSignal<Double> rightClimberErrorSignal;

  private final MotionMagicVoltage leftPositionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage rightPositionRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private final LinearSystemSim<N2, N1, N1> leftClimberSim;
  private final LinearSystemSim<N2, N1, N1> rightClimberSim;

  private double lastLeftPos = 0.0;
  private double lastRightPos = 0.0;

  public ClimberIOPhoenix() {
    leftMotor = new TalonFX(Constants.Climber.leftMotorID, Constants.canivoreBusName);
    rightMotor = new TalonFX(Constants.Climber.rightMotorID, Constants.canivoreBusName);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Climber.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.StatorCurrentLimit = 300.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.Slot0.kP = Constants.Climber.kP;
    motorConfig.Slot0.kI = Constants.Climber.kI;
    motorConfig.Slot0.kD = Constants.Climber.kD;
    // motorConfig.Slot0.kV = Constants.Climber.kV;
    // motorConfig.Slot0.kA = Constants.Climber.kA;
    motorConfig.Slot0.kS = Constants.Climber.kS;
    //    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    //    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.Climber.forwardLimit / Constants.Climber.metersPerRotation;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.Climber.reverseLimit / Constants.Climber.metersPerRotation;
    motorConfig.MotionMagic.MotionMagicExpo_kV = Constants.Climber.kV;
    motorConfig.MotionMagic.MotionMagicExpo_kA = Constants.Climber.kA;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.Climber.mmMaxVel;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.Climber.mmMaxAccel;
    motorConfig.Audio.AllowMusicDurDisable = true;

    leftMotor.getConfigurator().apply(motorConfig);
    rightMotor.getConfigurator().apply(motorConfig);

    leftClimberPositionSignal = leftMotor.getPosition();
    leftClimberVelocitySignal = leftMotor.getVelocity();
    leftClimberAccelSignal = leftMotor.getAcceleration();
    leftClimberMotorTempSignal = leftMotor.getDeviceTemp();
    leftClimberSupplyCurrentSignal = leftMotor.getSupplyCurrent();
    leftClimberStatorCurrentSignal = leftMotor.getStatorCurrent();
    leftClimberVoltageSignal = leftMotor.getMotorVoltage();
    leftClimberErrorSignal = leftMotor.getClosedLoopError();
    leftClimberSupplyVoltage = leftMotor.getSupplyVoltage();

    rightClimberPositionSignal = rightMotor.getPosition();
    rightClimberVelocitySignal = rightMotor.getVelocity();
    rightClimberAccelSignal = rightMotor.getAcceleration();
    rightClimberMotorTempSignal = rightMotor.getDeviceTemp();
    rightClimberSupplyCurrentSignal = rightMotor.getSupplyCurrent();
    rightClimberStatorCurrentSignal = rightMotor.getStatorCurrent();
    rightClimberVoltageSignal = rightMotor.getMotorVoltage();
    rightClimberSupplyVoltage = rightMotor.getSupplyVoltage();
    rightClimberErrorSignal = rightMotor.getClosedLoopError();

    this.leftClimberSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(Constants.Climber.kV, Constants.Climber.kA));

    this.rightClimberSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(Constants.Climber.kV, Constants.Climber.kA));

    leftMotorSim = leftMotor.getSimState();
    rightMotorSim = rightMotor.getSimState();

    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  /**
   * Update the inputs for the climber
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ClimberInputs inputs) {
    if (Robot.isSimulation()) {
      leftMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      rightMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double leftVoltage = leftMotorSim.getMotorVoltage();
      double rightVoltage = rightMotorSim.getMotorVoltage();

      leftClimberSim.setInput(leftVoltage);
      rightClimberSim.setInput(rightVoltage);
      leftClimberSim.update(0.02);
      rightClimberSim.update(0.02);

      double leftPosRot = leftClimberSim.getOutput(0);
      double leftVelRot = (leftPosRot - lastLeftPos) / 0.02;
      lastLeftPos = leftPosRot;

      double rightPosRot = rightClimberSim.getOutput(0);
      double rightVelRot = (rightPosRot - lastRightPos) / 0.02;
      lastRightPos = rightPosRot;

      leftMotorSim.setRawRotorPosition(leftPosRot);
      leftMotorSim.setRotorVelocity(leftVelRot);

      rightMotorSim.setRawRotorPosition(rightPosRot);
      rightMotorSim.setRotorVelocity(rightVelRot);
    }

    double leftClimberRot = leftClimberPositionSignal.getValue();
    double rightClimberRot = rightClimberPositionSignal.getValue();

    inputs.leftClimberLengthMeters = leftClimberRot * Constants.Climber.metersPerRotation;
    inputs.leftClimberVelocityMps =
        leftClimberVelocitySignal.getValue() * Constants.Climber.metersPerRotation;
    inputs.leftClimberAccelMpsSq =
        leftClimberAccelSignal.getValue() * Constants.Climber.metersPerRotation;
    inputs.leftClimberMotorTemp = leftClimberMotorTempSignal.getValue();
    inputs.leftClimberSupplyCurrent = leftClimberSupplyCurrentSignal.getValue();
    inputs.leftClimberStatorCurrent = leftClimberStatorCurrentSignal.getValue();
    inputs.leftClimberVoltage = leftClimberVoltageSignal.getValue();
    inputs.leftClimberSupplyVoltage = leftClimberSupplyVoltage.getValue();
    inputs.leftClimberClosedLoopError = leftClimberErrorSignal.getValue();

    inputs.rightClimberLengthMeters = rightClimberRot * Constants.Climber.metersPerRotation;
    inputs.rightClimberVelocityMps =
        rightClimberVelocitySignal.getValue() * Constants.Climber.metersPerRotation;
    inputs.rightClimberAccelMpsSq =
        rightClimberAccelSignal.getValue() * Constants.Climber.metersPerRotation;
    inputs.rightClimberMotorTemp = rightClimberMotorTempSignal.getValue();
    inputs.rightClimberSupplyCurrent = rightClimberSupplyCurrentSignal.getValue();
    inputs.rightClimberStatorCurrent = rightClimberStatorCurrentSignal.getValue();
    inputs.rightClimberVoltage = rightClimberVoltageSignal.getValue();
    inputs.rightClimberSupplyVoltage = rightClimberSupplyVoltage.getValue();
    inputs.rightClimberClosedLoopError = rightClimberErrorSignal.getValue();

    BaseStatusSignal.refreshAll(
        leftClimberPositionSignal,
        leftClimberVelocitySignal,
        leftClimberAccelSignal,
        leftClimberMotorTempSignal,
        leftClimberSupplyCurrentSignal,
        leftClimberStatorCurrentSignal,
        leftClimberVoltageSignal,
        leftClimberErrorSignal,
        leftClimberSupplyVoltage,
        rightClimberPositionSignal,
        rightClimberVelocitySignal,
        rightClimberAccelSignal,
        rightClimberMotorTempSignal,
        rightClimberSupplyCurrentSignal,
        rightClimberStatorCurrentSignal,
        rightClimberVoltageSignal,
        rightClimberSupplyVoltage,
        rightClimberErrorSignal);
  }

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  @Override
  public void setLeftClimberLength(double length) {
    leftMotor.setControl(
        leftPositionRequest.withPosition(length / Constants.Climber.metersPerRotation));
  }

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  @Override
  public void setRightClimberLength(double length) {
    rightMotor.setControl(
        rightPositionRequest.withPosition(length / Constants.Climber.metersPerRotation));
  }

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setLeftVoltage(double volts) {
    leftMotor.setControl(leftVoltageRequest.withOutput(volts));
  }

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setRightVoltage(double volts) {
    rightMotor.setControl(rightVoltageRequest.withOutput(volts));
  }

  @Override
  public void resetPosition() {
    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  @Override
  public void brakeMotors() {
    leftMotor.setControl(brakeRequest);
    rightMotor.setControl(brakeRequest);
  }

  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        leftMotor.getPosition(),
        leftMotor.getVelocity(),
        leftMotor.getMotorVoltage(),
        rightMotor.getPosition(),
        rightMotor.getVelocity(),
        rightMotor.getMotorVoltage());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(leftMotor, rightMotor);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Left Climber Motor", leftMotor);
    subsystem.registerHardware("Right Climber Motor", rightMotor);
  }
}
