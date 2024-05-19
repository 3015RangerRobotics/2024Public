package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  class ShooterInputs {
    public double topMotorVel = 0.0;
    public double topMotorAccel = 0.0;
    public double topMotorTemp = 0.0;
    public double topMotorSupplyCurrent = 0.0;
    public double topMotorStatorCurrent = 0.0;
    public double topMotorVoltage = 0.0;
    public double topMotorSupplyVoltage = 0.0;
    public double topMotorClosedLoopError = 0.0;

    public double bottomMotorVel = 0.0;
    public double bottomMotorAccel = 0.0;
    public double bottomMotorTemp = 0.0;
    public double bottomMotorSupplyCurrent = 0.0;
    public double bottomMotorStatorCurrent = 0.0;
    public double bottomMotorVoltage = 0.0;
    public double bottomMotorSupplyVoltage = 0.0;
    public double bottomMotorClosedLoopError = 0.0;
  }

  void updateInputs(ShooterInputs inputs);

  void setTargetMotorVel(double topMotorVel, double bottomMotorVel);

  void setVoltage(double topVolts, double bottomVolts);

  void stopMotors();

  default void optimizeForSysID() {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
