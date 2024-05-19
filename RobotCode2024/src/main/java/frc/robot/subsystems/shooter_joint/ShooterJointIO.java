package frc.robot.subsystems.shooter_joint;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterJointIO {
  @AutoLog
  class ShooterJointInputs {
    public double jointAngleDegrees = 0.0;
    public double jointVelocityDps = 0.0;
    public double jointAccelDpsSq = 0.0;

    public double jointMotorTemp = 0.0;
    public double jointMotorSupplyCurrent = 0.0;
    public double jointMotorStatorCurrent = 0.0;
    public double jointMotorVoltage = 0.0;
    public double jointMotorSupplyVoltage = 0.0;
    public double jointMotorClosedLoopError = 0.0;
  }

  void updateInputs(ShooterJointInputs inputs);

  void setTargetAngle(Rotation2d angle);

  void setTargetAngleProfiled(Rotation2d angle);

  void setVoltage(double volts);

  void brakeMotor();

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  default void optimizeForSysID() {}

  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }
}
