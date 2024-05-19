package frc.robot.subsystems.swerve_module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.lib.subsystem.AdvancedSubsystem;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
  @AutoLog
  class SwerveModuleOdometryInputs {
    public double drivePositionRot = 0.0;
    public double driveVelocityRps = 0.0;

    public double rotationPositionDegrees = 0.0;
    public double rotationVelocityDps = 0.0;
  }

  @AutoLog
  class SwerveModuleMiscInputs {
    public double driveTemperature = 0.0;
    public double driveAccelMpsSq = 0.0;
    public double driveSupplyCurrent = 0.0;
    public double driveStatorCurrent = 0.0;
    public double driveVoltage = 0.0;
    public double driveSupplyVoltage = 0.0;
    public double driveClosedLoopError = 0.0;

    public double rotationTemperature = 0.0;
    public double rotationAccelDpsSq = 0.0;
    public double rotationSupplyCurrent = 0.0;
    public double rotationStatorCurrent = 0.0;
    public double rotationVoltage = 0.0;
    public double rotationSupplyVoltage = 0.0;
    public double rotationClosedLoopError = 0.0;
  }

  /**
   * Update the odometry inputs. This should not refresh any signals, as that will be handled in the
   * odometry thread
   */
  void updateOdometryInputs(SwerveModuleOdometryInputs inputs);

  /** Update the misc inputs. This should refresh signals. */
  void updateMiscInputs(SwerveModuleMiscInputs inputs);

  void setTargetDriveVelocity(double driveVelocityMps);

  void setTargetRotation(double rotationDegrees);

  void setDriveVoltage(double volts);

  void setRotationVoltage(double volts);

  void stopMotors();

  void brakeDrive();

  void brakeRotation();

  void registerSelfCheckHardware(AdvancedSubsystem subsystem);

  BaseStatusSignal[] getOdometrySignals();

  List<ParentDevice> getOrchestraDevices();
}
