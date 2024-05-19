package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;

public class IntakeIOPhoenix implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFXSimState intakeMotorSim;

  private final StatusSignal<Double> intakeVelocitySignal;
  private final StatusSignal<Double> intakeAccelSignal;
  private final StatusSignal<Double> intakeMotorTempSignal;
  private final StatusSignal<Double> intakeMotorSupplyCurrentSignal;
  private final StatusSignal<Double> intakeMotorStatorCurrentSignal;
  private final StatusSignal<Double> intakeMotorVoltageSignal;

  private final StatusSignal<Double> intakeMotorSupplyVoltage;
  private final StatusSignal<Double> intakeErrorSignal;

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  private final LinearSystemSim<N1, N1, N1> intakeSim;

  public IntakeIOPhoenix() {
    intakeMotor = new TalonFX(Constants.Intake.motorID, Constants.canivoreBusName);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Slot0.kP = Constants.Intake.kP;
    motorConfig.Slot0.kI = Constants.Intake.kI;
    motorConfig.Slot0.kD = Constants.Intake.kD;
    motorConfig.Slot0.kV = Constants.Intake.kV;
    motorConfig.Slot0.kA = Constants.Intake.kA;
    motorConfig.Slot0.kS = Constants.Intake.kS;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Intake.currentLimit;
    motorConfig.Audio.AllowMusicDurDisable = true;

    intakeMotor.getConfigurator().apply(motorConfig);

    intakeVelocitySignal = intakeMotor.getVelocity();
    intakeAccelSignal = intakeMotor.getAcceleration();
    intakeMotorTempSignal = intakeMotor.getDeviceTemp();
    intakeMotorSupplyCurrentSignal = intakeMotor.getSupplyCurrent();
    intakeMotorStatorCurrentSignal = intakeMotor.getStatorCurrent();
    intakeMotorVoltageSignal = intakeMotor.getMotorVoltage();
    intakeMotorSupplyVoltage = intakeMotor.getSupplyVoltage();
    intakeErrorSignal = intakeMotor.getClosedLoopError();

    intakeSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyVelocitySystem(Constants.Intake.kV, Constants.Intake.kA));

    intakeMotorSim = intakeMotor.getSimState();
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (Robot.isSimulation()) {
      intakeMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double intakeVoltage = intakeMotorSim.getMotorVoltage();

      intakeSim.setInput(intakeVoltage);
      intakeSim.update(0.02);

      intakeMotorSim.setRotorVelocity(intakeSim.getOutput(0));
    }

    inputs.intakeMotorVel = intakeVelocitySignal.getValue();
    inputs.intakeMotorAccel = intakeAccelSignal.getValue();
    inputs.intakeMotorTemp = intakeMotorTempSignal.getValue();
    inputs.intakeMotorSupplyCurrent = intakeMotorSupplyCurrentSignal.getValue();
    inputs.intakeMotorStatorCurrent = intakeMotorStatorCurrentSignal.getValue();
    inputs.intakeMotorVoltage = intakeMotorVoltageSignal.getValue();
    inputs.intakeMotorSupplyVoltage = intakeMotorSupplyVoltage.getValue();
    inputs.intakeClosedLoopError = intakeErrorSignal.getValue();

    BaseStatusSignal.refreshAll(
        intakeVelocitySignal,
        intakeAccelSignal,
        intakeMotorTempSignal,
        intakeMotorSupplyCurrentSignal,
        intakeMotorStatorCurrentSignal,
        intakeMotorVoltageSignal,
        intakeMotorSupplyVoltage,
        intakeErrorSignal);
  }

  @Override
  public void setTargetMotorVel(double motorVelocity) {
    intakeMotor.setControl(velocityRequest.withVelocity(motorVelocity));
  }

  @Override
  public void setVoltage(double voltage) {
    intakeMotor.setControl(voltageRequest.withOutput(voltage));
  }

  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Intake Motor", intakeMotor);
  }

  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, intakeMotor.getPosition(), intakeMotor.getVelocity(), intakeMotor.getMotorVoltage());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(intakeMotor);
  }
}
