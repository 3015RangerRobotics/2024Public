package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.subsystem.AdvancedSubsystem;
import org.littletonrobotics.junction.Logger;

public class SwerveIOSim implements SwerveIO {
  private double currentAngularVel = 0.0;
  private double lastYaw = 0.0;
  private double lastOdomTime = Timer.getFPGATimestamp();

  @Override
  public void updateIMUOdomInputs(IMUOdomInputs inputs) {
    inputs.measurementTimestamp = Logger.getRealTimestamp() / 1000000.0;
    double dt = inputs.measurementTimestamp - lastOdomTime;
    lastOdomTime = inputs.measurementTimestamp;

    inputs.yawDeg = lastYaw + (currentAngularVel * dt);
    lastYaw = inputs.yawDeg;

    inputs.angularVelZ = currentAngularVel;

    inputs.rollDeg = 0.0;
    inputs.pitchDeg = 0.0;
    inputs.angularVelX = 0.0;
    inputs.angularVelY = 0.0;
  }

  @Override
  public void updateIMUMiscInputs(IMUMiscInputs inputs) {
    inputs.gravVectorX = 0;
    inputs.gravVectorY = 0;
    inputs.gravVectorZ = -1;
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[0];
  }

  @Override
  public void setCurrentAngularVel(double currentAngularVel) {
    this.currentAngularVel = currentAngularVel;
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}
}
