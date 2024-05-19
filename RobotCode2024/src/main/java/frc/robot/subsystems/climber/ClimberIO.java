package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  class ClimberInputs {
    public double leftClimberLengthMeters = 0.0;
    public double leftClimberVelocityMps = 0.0;
    public double leftClimberAccelMpsSq = 0.0;

    public double rightClimberLengthMeters = 0.0;
    public double rightClimberVelocityMps = 0.0;
    public double rightClimberAccelMpsSq = 0.0;

    public double leftClimberMotorTemp = 0.0;
    public double leftClimberSupplyCurrent = 0.0;
    public double leftClimberStatorCurrent = 0.0;
    public double leftClimberVoltage = 0.0;
    public double leftClimberSupplyVoltage = 0.0;
    public double leftClimberClosedLoopError = 0.0;

    public double rightClimberMotorTemp = 0.0;
    public double rightClimberSupplyCurrent = 0.0;
    public double rightClimberStatorCurrent = 0.0;
    public double rightClimberVoltage = 0.0;
    public double rightClimberSupplyVoltage = 0.0;
    public double rightClimberClosedLoopError = 0.0;
  }

  /**
   * Update the inputs for the climber
   *
   * @param inputs The inputs to update
   */
  void updateInputs(ClimberInputs inputs);

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  void setLeftClimberLength(double length);

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  void setRightClimberLength(double length);

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  void setLeftVoltage(double volts);

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  void setRightVoltage(double volts);

  void brakeMotors();

  default void resetPosition() {}

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

  default void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
