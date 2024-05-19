package frc.robot.subsystems.arm_extension;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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

public class ArmExtensionIOPhoenix implements ArmExtensionIO {
  private final TalonFX armExtensionMotor;
  private final TalonFXSimState armExtensionMotorSim;

  private final StatusSignal<Double> armExtensionPositionSignal;
  private final StatusSignal<Double> armExtensionVelocitySignal;
  private final StatusSignal<Double> armExtensionAccelSignal;
  private final StatusSignal<Double> armExtensionMotorTempSignal;
  private final StatusSignal<Double> armExtensionSupplyCurrentSignal;
  private final StatusSignal<Double> armExtensionStatorCurrentSignal;
  private final StatusSignal<Double> armExtensionVoltageSignal;
  private final StatusSignal<Double> armExtensionErrorSignal;
  private final StatusSignal<Double> armExtensionSupplyVoltage;

  private final LinearSystemSim<N2, N1, N1> extensionSim;

  private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final StaticBrake brakeRequest = new StaticBrake();

  private double lastExtensionPos = 0.0;

  public ArmExtensionIOPhoenix() {
    armExtensionMotor = new TalonFX(Constants.ArmExtension.motorID, Constants.canivoreBusName);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.ArmExtension.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Slot0.kP = Constants.ArmExtension.kP;
    motorConfig.Slot0.kI = Constants.ArmExtension.kI;
    motorConfig.Slot0.kD = Constants.ArmExtension.kD;
    //    motorConfig.Slot0.kV = Constants.ArmExtension.kV;
    //    motorConfig.Slot0.kA = Constants.ArmExtension.kA;
    motorConfig.Slot0.kS = Constants.ArmExtension.kS;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Constants.ArmExtension.forwardLimit / Constants.ArmExtension.metersPerRotation;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Constants.ArmExtension.reverseLimit / Constants.ArmExtension.metersPerRotation;
    motorConfig.MotionMagic.MotionMagicExpo_kV = Constants.ArmExtension.kV;
    motorConfig.MotionMagic.MotionMagicExpo_kA = Constants.ArmExtension.kA;
    motorConfig.MotionMagic.MotionMagicCruiseVelocity = Constants.ArmExtension.mmMaxVel;
    motorConfig.MotionMagic.MotionMagicAcceleration = Constants.ArmExtension.mmMaxAccel;
    motorConfig.MotionMagic.MotionMagicJerk = 0.0; // Constants.ArmExtension.mmMaxJerk;
    motorConfig.Audio.AllowMusicDurDisable = true;

    armExtensionMotor.getConfigurator().apply(motorConfig);

    armExtensionPositionSignal = armExtensionMotor.getPosition();
    armExtensionVelocitySignal = armExtensionMotor.getVelocity();
    armExtensionAccelSignal = armExtensionMotor.getAcceleration();
    armExtensionMotorTempSignal = armExtensionMotor.getDeviceTemp();
    armExtensionSupplyCurrentSignal = armExtensionMotor.getSupplyCurrent();
    armExtensionStatorCurrentSignal = armExtensionMotor.getStatorCurrent();
    armExtensionVoltageSignal = armExtensionMotor.getMotorVoltage();
    armExtensionErrorSignal = armExtensionMotor.getClosedLoopError();
    armExtensionSupplyVoltage = armExtensionMotor.getSupplyVoltage();

    this.extensionSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(
                Constants.ArmExtension.kV, Constants.ArmExtension.kA));

    armExtensionMotorSim = armExtensionMotor.getSimState();
  }

  /**
   * Update the inputs for the arm extension
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmExtensionInputs inputs) {
    if (Robot.isSimulation()) {
      armExtensionMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double armVoltage = armExtensionMotorSim.getMotorVoltage();

      extensionSim.setInput(armVoltage);
      extensionSim.update(0.02);

      double extensionPosRot = extensionSim.getOutput(0);
      double extensionVelRot = (extensionPosRot - lastExtensionPos) / 0.02;
      lastExtensionPos = extensionPosRot;

      armExtensionMotorSim.setRawRotorPosition(extensionPosRot);
      armExtensionMotorSim.setRotorVelocity(extensionVelRot);
    }

    double armExt = armExtensionPositionSignal.getValue();

    inputs.extensionMeters = armExt * Constants.ArmExtension.metersPerRotation;
    inputs.velocityMps =
        armExtensionVelocitySignal.getValue() * Constants.ArmExtension.metersPerRotation;
    inputs.accelMpsSq =
        armExtensionAccelSignal.getValue() * Constants.ArmExtension.metersPerRotation;
    inputs.armExtensionMotorTemp = armExtensionMotorTempSignal.getValue();
    inputs.armExtensionSupplyCurrent = armExtensionSupplyCurrentSignal.getValue();
    inputs.armExtensionStatorCurrent = armExtensionStatorCurrentSignal.getValue();
    inputs.armExtensionVoltage = armExtensionVoltageSignal.getValue();
    inputs.armExtensionClosedLoopError =
        armExtensionErrorSignal.getValue() * Constants.ArmExtension.metersPerRotation;
    inputs.armExtensionSupplyVoltage = armExtensionSupplyVoltage.getValue();

    BaseStatusSignal.refreshAll(
        armExtensionPositionSignal,
        armExtensionVelocitySignal,
        armExtensionAccelSignal,
        armExtensionMotorTempSignal,
        armExtensionSupplyCurrentSignal,
        armExtensionStatorCurrentSignal,
        armExtensionVoltageSignal,
        armExtensionErrorSignal,
        armExtensionSupplyVoltage);
  }

  /**
   * Set the target extension length
   *
   * @param lengthMeters Target extension length, in meters
   */
  @Override
  public void setTargetExtension(double lengthMeters) {
    armExtensionMotor.setControl(
        positionRequest.withPosition(lengthMeters / Constants.ArmExtension.metersPerRotation));
  }

  /**
   * Output a set voltage to the arm extension motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {
    armExtensionMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void brakeMotor() {
    armExtensionMotor.setControl(brakeRequest);
  }

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Extension Motor", armExtensionMotor);
  }

  /**
   * Get a list of all devices to be used for orchestra commands
   *
   * @return Orchestra compatible CTRE devices
   */
  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(armExtensionMotor);
  }

  /** Optimize status signals for running sysID */
  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        armExtensionMotor.getPosition(),
        armExtensionMotor.getVelocity(),
        armExtensionMotor.getMotorVoltage());
  }
}
