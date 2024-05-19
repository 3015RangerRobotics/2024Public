package frc.robot.subsystems.uptake;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface UptakeIO {
  @AutoLog
  class UptakeInputs {
    public double motorPos = 0.0;
    public double motorVel = 0.0;
    public double motorAccel = 0.0;

    public boolean ringSensor = false;

    public double motorTemp = 0.0;
    public double motorSupplyCurrent = 0.0;
    public double motorStatorCurrent = 0.0;
    public double motorVoltage = 0.0;
    public double motorSupplyVoltage = 0.0;
    public double motorClosedLoopError = 0.0;
  }

  void updateInputs(UptakeInputs inputs);

  void setTargetMotorVel(double motorVel);

  void setTargetMotorPos(double motorPos);

  void setVoltage(double volts);

  default void configForwardLimit(boolean enable) {}

  void stopMotor();

  default void optimizeForSysID() {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
