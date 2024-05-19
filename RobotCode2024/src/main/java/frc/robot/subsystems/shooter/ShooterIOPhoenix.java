package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
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

public class ShooterIOPhoenix implements ShooterIO {
  private final TalonFX topMotor;
  private final TalonFXSimState topMotorSim;
  private final TalonFX bottomMotor;
  private final TalonFXSimState bottomMotorSim;

  private final StatusSignal<Double> topVelocitySignal;
  private final StatusSignal<Double> topAccelSignal;
  private final StatusSignal<Double> topTempSignal;
  private final StatusSignal<Double> topSupplyCurrentSignal;
  private final StatusSignal<Double> topStatorCurrentSignal;
  private final StatusSignal<Double> topVoltageSignal;

  private final StatusSignal<Double> topMotorSupplyVoltage;
  private final StatusSignal<Double> topErrorSignal;

  private final StatusSignal<Double> bottomVelocitySignal;
  private final StatusSignal<Double> bottomAccelSignal;
  private final StatusSignal<Double> bottomTempSignal;
  private final StatusSignal<Double> bottomSupplyCurrentSignal;
  private final StatusSignal<Double> bottomStatorCurrentSignal;
  private final StatusSignal<Double> bottomVoltageSignal;

  private final StatusSignal<Double> bottomMotorSupplyVoltage;
  private final StatusSignal<Double> bottomErrorSignal;

  private final VelocityVoltage topVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private final VelocityVoltage bottomVelocityRequest = new VelocityVoltage(0).withEnableFOC(false);
  private final VoltageOut topVoltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final VoltageOut bottomVoltageRequest = new VoltageOut(0).withEnableFOC(false);
  private final NeutralOut neutralRequest = new NeutralOut();

  private final LinearSystemSim<N1, N1, N1> topShooterSim;
  private final LinearSystemSim<N1, N1, N1> bottomShooterSim;

  public ShooterIOPhoenix() {
    topMotor = new TalonFX(Constants.Shooter.topMotorID, Constants.canivoreBusName);
    bottomMotor = new TalonFX(Constants.Shooter.bottomMotorID, Constants.canivoreBusName);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorConfig.Slot0.kP = Constants.Shooter.kP;
    motorConfig.Slot0.kI = Constants.Shooter.kI;
    motorConfig.Slot0.kD = Constants.Shooter.kD;
    motorConfig.Slot0.kV = Constants.Shooter.kV;
    motorConfig.Slot0.kA = Constants.Shooter.kA;
    motorConfig.Slot0.kS = Constants.Shooter.kS;
    motorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Shooter.currentLimit;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    //    motorConfig.TorqueCurrent.PeakForwardTorqueCurrent = Constants.Shooter.currentLimit;
    //    motorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -Constants.Shooter.currentLimit;
    motorConfig.Audio.AllowMusicDurDisable = true;

    topMotor.getConfigurator().apply(motorConfig);

    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    bottomMotor.getConfigurator().apply(motorConfig);

    topVelocitySignal = topMotor.getVelocity();
    topAccelSignal = topMotor.getAcceleration();
    topTempSignal = topMotor.getDeviceTemp();
    topSupplyCurrentSignal = topMotor.getSupplyCurrent();
    topStatorCurrentSignal = topMotor.getStatorCurrent();
    topVoltageSignal = topMotor.getMotorVoltage();
    topMotorSupplyVoltage = topMotor.getSupplyVoltage();
    topErrorSignal = topMotor.getClosedLoopError();

    bottomVelocitySignal = bottomMotor.getVelocity();
    bottomAccelSignal = bottomMotor.getAcceleration();
    bottomTempSignal = bottomMotor.getDeviceTemp();
    bottomSupplyCurrentSignal = bottomMotor.getSupplyCurrent();
    bottomStatorCurrentSignal = bottomMotor.getStatorCurrent();
    bottomVoltageSignal = bottomMotor.getMotorVoltage();
    bottomMotorSupplyVoltage = bottomMotor.getSupplyVoltage();
    bottomErrorSignal = bottomMotor.getClosedLoopError();

    this.topShooterSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyVelocitySystem(Constants.Shooter.kV, Constants.Shooter.kA));
    this.bottomShooterSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyVelocitySystem(Constants.Shooter.kV, Constants.Shooter.kA));

    topMotorSim = topMotor.getSimState();
    bottomMotorSim = bottomMotor.getSimState();
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    if (Robot.isSimulation()) {
      topMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
      bottomMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

      double topVoltage = topMotorSim.getMotorVoltage();
      double bottomVoltage = bottomMotorSim.getMotorVoltage();

      topShooterSim.setInput(topVoltage);
      bottomShooterSim.setInput(bottomVoltage);

      topShooterSim.update(0.02);
      bottomShooterSim.update(0.02);

      topMotorSim.setRotorVelocity(topShooterSim.getOutput(0));
      bottomMotorSim.setRotorVelocity(bottomShooterSim.getOutput(0));
    }

    inputs.topMotorVel = topVelocitySignal.getValue();
    inputs.topMotorAccel = topAccelSignal.getValue();
    inputs.topMotorTemp = topTempSignal.getValue();
    inputs.topMotorSupplyCurrent = topSupplyCurrentSignal.getValue();
    inputs.topMotorStatorCurrent = topStatorCurrentSignal.getValue();
    inputs.topMotorVoltage = topVoltageSignal.getValue();
    inputs.topMotorSupplyVoltage = topMotorSupplyVoltage.getValue();
    inputs.topMotorClosedLoopError = topErrorSignal.getValue();

    inputs.bottomMotorVel = bottomVelocitySignal.getValue();
    inputs.bottomMotorAccel = bottomAccelSignal.getValue();
    inputs.bottomMotorTemp = bottomTempSignal.getValue();
    inputs.bottomMotorSupplyCurrent = bottomSupplyCurrentSignal.getValue();
    inputs.bottomMotorStatorCurrent = bottomStatorCurrentSignal.getValue();
    inputs.bottomMotorVoltage = bottomVoltageSignal.getValue();
    inputs.bottomMotorSupplyVoltage = bottomMotorSupplyVoltage.getValue();
    inputs.bottomMotorClosedLoopError = bottomErrorSignal.getValue();

    BaseStatusSignal.refreshAll(
        topVelocitySignal,
        topAccelSignal,
        topTempSignal,
        topSupplyCurrentSignal,
        topStatorCurrentSignal,
        topVoltageSignal,
        topMotorSupplyVoltage,
        topErrorSignal,
        bottomVelocitySignal,
        bottomAccelSignal,
        bottomTempSignal,
        bottomSupplyCurrentSignal,
        bottomStatorCurrentSignal,
        bottomVoltageSignal,
        bottomMotorSupplyVoltage,
        bottomErrorSignal);
  }

  @Override
  public void setTargetMotorVel(double topMotorVel, double bottomMotorVel) {
    topMotor.setControl(topVelocityRequest.withVelocity(topMotorVel));
    bottomMotor.setControl(bottomVelocityRequest.withVelocity(bottomMotorVel));
  }

  @Override
  public void setVoltage(double topVolts, double bottomVolts) {
    topMotor.setControl(topVoltageRequest.withOutput(topVolts));
    bottomMotor.setControl(bottomVoltageRequest.withOutput(bottomVolts));
  }

  @Override
  public void stopMotors() {
    topMotor.setControl(neutralRequest);
    bottomMotor.setControl(neutralRequest);
  }

  @Override
  public void optimizeForSysID() {
    BaseStatusSignal.setUpdateFrequencyForAll(
        250,
        topMotor.getPosition(),
        topMotor.getVelocity(),
        topMotor.getMotorVoltage(),
        bottomMotor.getPosition(),
        bottomMotor.getVelocity(),
        bottomMotor.getMotorVoltage());
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return List.of(topMotor, bottomMotor);
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("Top Shooter", topMotor);
    subsystem.registerHardware("Bottom Shooter", bottomMotor);
  }
}
