package frc.robot.subsystems.arm_joint;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ArmJointIO {
  @AutoLog
  class ArmJointInputs {
    public double jointAngleDegrees = 0.0;
    public double jointVelocityDps = 0.0;
    public double jointAccelDpsSq = 0.0;
    public double jointClosedLoopError = 0.0;

    public double jointMotorTemp = 0.0;
    public double jointMotorSupplyCurrent = 0.0;
    public double jointMotorStatorCurrent = 0.0;
    public double jointMotorVoltage = 0.0;
    public double jointMotorSupplyVoltage = 0.0;

    public double jointFollowerTemp = 0.0;
    public double jointFollowerSupplyCurrent = 0.0;

    public double jointFollowerStatorCurrent = 0.0;
    public double jointFollowerVoltage = 0.0;
  }

  /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  void updateInputs(ArmJointInputs inputs);

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  void setTargetAngle(Rotation2d angle);

  void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel);

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  void setVoltage(double volts);

  void brakeMotors();

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  /** Optimize status signals for running sysID */
  default void optimizeForSysID() {}

  /**
   * Get a list of all devices to be used for orchestra commands
   *
   * @return Orchestra compatible CTRE devices
   */
  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }
}
