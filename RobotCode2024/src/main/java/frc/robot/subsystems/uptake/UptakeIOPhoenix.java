package frc.robot.subsystems.uptake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
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

public class UptakeIOPhoenix implements UptakeIO {
  private final TalonFX uptakeMotor;
  private final TalonFXSimState motorSim;

  private final StatusSignal<Double> positionSignal;
  private final StatusSignal<Double> velocitySignal;
  private final StatusSignal<Double> accelSignal;
  private final StatusSignal<Double> tempSignal;
  private final StatusSignal<Double> supplyCurrentSignal;
  private final StatusSignal<Double> statorCurrentSignal;
  private final StatusSignal<Double> voltageSignal;

  private final StatusSignal<Double> motorSupplyVoltage;
  private final StatusSignal<Double> errorSignal;
  private final StatusSignal<ForwardLimitValue> ringSensorSignal;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(1);
  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final LinearSystemSim<N2, N1, N1> uptakeSim;
  private double lastMotorPos = 0.0;

  public UptakeIOPhoenix() {
    uptakeMotor = new TalonFX(Constants.Uptake.motorID, Constants.canivoreBusName);

    // Motor Config
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    motorConfig.Slot0.kP = Constants.Uptake.vel_kP;
    motorConfig.Slot0.kI = Constants.Uptake.vel_kI;
    motorConfig.Slot0.kD = Constants.Uptake.vel_kD;
    motorConfig.Slot0.kV = Constants.Uptake.kV;
    motorConfig.Slot0.kA = Constants.Uptake.kA;
    motorConfig.Slot0.kS = Constants.Uptake.kS;
    motorConfig.Slot1.kP = Constants.Uptake.pos_kP;
    motorConfig.Slot1.kI = Constants.Uptake.pos_kI;
    motorConfig.Slot1.kD = Constants.Uptake.pos_kD;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Uptake.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.Audio.AllowMusicDurDisable = true;
    motorConfig.HardwareLimitSwitch.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    motorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    motorConfig.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfig.Audio.BeepOnConfig = false;

    uptakeMotor.getConfigurator().apply(motorConfig);

    positionSignal = uptakeMotor.getPosition();
    velocitySignal = uptakeMotor.getVelocity();
    accelSignal = uptakeMotor.getAcceleration();
    tempSignal = uptakeMotor.getDeviceTemp();
    supplyCurrentSignal = uptakeMotor.getSupplyCurrent();
    statorCurrentSignal = uptakeMotor.getStatorCurrent();
    voltageSignal = uptakeMotor.getMotorVoltage();
    motorSupplyVoltage = uptakeMotor.getSupplyVoltage();
    errorSignal = uptakeMotor.getClosedLoopError();
    ringSensorSignal = uptakeMotor.getForwardLimit();

    this.uptakeSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(Constants.Uptake.kV, Constants.Uptake.kA));

    motorSim = uptakeMotor.getSimState();
  }

  @Override
  public void updateInputs(UptakeIO.UptakeInputs inputs) {
    if (Robot.isSimulation()) {
      motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double voltage = motorSim.getMotorVoltage();

      uptakeSim.setInput(voltage);
      uptakeSim.update(0.02);

      double motorPos = uptakeSim.getOutput(0);
      double motorVel = (motorPos - lastMotorPos) / 0.02;
      lastMotorPos = motorPos;

      motorSim.setRawRotorPosition(motorPos);
      motorSim.setRotorVelocity(motorVel);
    }

    inputs.motorPos = positionSignal.getValue();
    inputs.motorVel = velocitySignal.getValue();
    inputs.motorAccel = accelSignal.getValue();
    inputs.ringSensor = ringSensorSignal.getValue() == ForwardLimitValue.ClosedToGround;
    inputs.motorTemp = tempSignal.getValue();
    inputs.motorSupplyCurrent = supplyCurrentSignal.getValue();
    inputs.motorStatorCurrent = statorCurrentSignal.getValue();
    inputs.motorVoltage = voltageSignal.getValue();
    inputs.motorSupplyVoltage = motorSupplyVoltage.getValue();
    inputs.motorClosedLoopError = errorSignal.getValue();

    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        accelSignal,
        tempSignal,
        supplyCurrentSignal,
        statorCurrentSignal,
        voltageSignal,
        errorSignal,
        motorSupplyVoltage,
        ringSensorSignal);
  }

  @Override
  public void setTargetMotorVel(double motorVel) {
    uptakeMotor.setControl(velocityRequest.withVelocity(motorVel));
  }

  @Override
  public void setTargetMotorPos(double motorPos) {
    uptakeMotor.setControl(positionRequest.withPosition(motorPos));
  }

  @Override
  public void setVoltage(double volts) {
    uptakeMotor.setControl(voltageRequest.withOutput((volts)));
  }

  @Override
  public void configForwardLimit(boolean enable) {
    HardwareLimitSwitchConfigs config = new HardwareLimitSwitchConfigs();
    config.ForwardLimitEnable = enable;
    config.ForwardLimitType = ForwardLimitTypeValue.NormallyOpen;
    config.ForwardLimitSource = ForwardLimitSourceValue.LimitSwitchPin;
    uptakeMotor.getConfigurator().apply(config, 0);
  }

  @Override
  public void stopMotor() {
    uptakeMotor.setControl(new StaticBrake());
  }

  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, uptakeMotor.getPosition(), uptakeMotor.getVelocity(), uptakeMotor.getMotorVoltage());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(uptakeMotor);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Uptake Motor", uptakeMotor);
  }
}
