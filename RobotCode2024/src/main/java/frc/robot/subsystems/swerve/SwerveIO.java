package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.lib.subsystem.AdvancedSubsystem;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveIO {
  @AutoLog
  class IMUOdomInputs {
    public double rollDeg = 0.0;
    public double pitchDeg = 0.0;
    public double yawDeg = 0.0;
    public double angularVelX = 0.0;
    public double angularVelY = 0.0;
    public double angularVelZ = 0.0;
    public double measurementTimestamp = 0.0;
  }

  @AutoLog
  class IMUMiscInputs {
    public double gravVectorX = 0.0;
    public double gravVectorY = 0.0;
    public double gravVectorZ = 0.0;
    public double accelX = 0.0;
    public double accelY = 0.0;
    public double accelZ = 0.0;
  }

  void updateIMUOdomInputs(IMUOdomInputs inputs);

  void updateIMUMiscInputs(IMUMiscInputs inputs);

  BaseStatusSignal[] getOdomSignals();

  void registerSelfCheckHardware(AdvancedSubsystem subsystem);

  default void setCurrentAngularVel(double angularVel) {}
}
