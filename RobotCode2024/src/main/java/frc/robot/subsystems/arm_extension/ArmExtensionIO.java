package frc.robot.subsystems.arm_extension;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ArmExtensionIO {
  @AutoLog
  class ArmExtensionInputs {
    public double extensionMeters = 0.0;
    public double velocityMps = 0.0;
    public double accelMpsSq = 0.0;

    public double armExtensionMotorTemp = 0.0;
    public double armExtensionSupplyCurrent = 0.0;
    public double armExtensionStatorCurrent = 0.0;
    public double armExtensionVoltage = 0.0;
    public double armExtensionClosedLoopError = 0.0;
    public double armExtensionSupplyVoltage = 0.0;
  }

  /**
   * Update the inputs for the arm extension
   *
   * @param inputs The inputs to update
   */
  void updateInputs(ArmExtensionInputs inputs);

  /**
   * Set the target extension length
   *
   * @param lengthMeters Target extension length, in meters
   */
  void setTargetExtension(double lengthMeters);

  /**
   * Output a set voltage to the arm extension motor
   *
   * @param volts Voltage to output
   */
  void setVoltage(double volts);

  void brakeMotor();

  /**
   * Register self check compatible hardware with its associated subsystem
   *
   * @param subsystem The subsystem to register hardware on
   */
  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  /**
   * Get a list of all devices to be used for orchestra commands
   *
   * @return Orchestra compatible CTRE devices
   */
  default List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  /** Optimize status signals for running sysID */
  default void optimizeForSysID() {}
}
