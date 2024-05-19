package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeInputs {
    public double intakeMotorVel = 0.0;

    public double intakeMotorAccel = 0.0;
    public double intakeMotorTemp = 0.0;
    public double intakeMotorSupplyCurrent = 0.0;
    public double intakeMotorStatorCurrent = 0.0;
    public double intakeMotorVoltage = 0.0;
    public double intakeMotorSupplyVoltage = 0.0;
    public double intakeClosedLoopError = 0.0;
  }

  void updateInputs(IntakeInputs inputs);

  void setTargetMotorVel(double motorVelocity);

  void setVoltage(double voltage);

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  default void optimizeForSysID() {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }
}
