package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SwerveIOPhoenix implements SwerveIO {
  private final Pigeon2 imu;
  // Odom inputs
  private final StatusSignal<Double> imuRollSignal;
  private final StatusSignal<Double> imuPitchSignal;
  private final StatusSignal<Double> imuYawSignal;
  private final StatusSignal<Double> imuAngularVelXSignal;
  private final StatusSignal<Double> imuAngularVelYSignal;
  private final StatusSignal<Double> imuAngularVelZSignal;
  // Misc inputs
  private final StatusSignal<Double> imuGravVectorXSignal;
  private final StatusSignal<Double> imuGravVectorYSignal;
  private final StatusSignal<Double> imuGravVectorZSignal;
  private final StatusSignal<Double> imuAccelXSignal;
  private final StatusSignal<Double> imuAccelYSignal;
  private final StatusSignal<Double> imuAccelZSignal;

  public SwerveIOPhoenix() {
    imu = new Pigeon2(Constants.Swerve.imuCanID, Constants.canivoreBusName);

    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPose.MountPoseYaw = -90.0;
    imu.getConfigurator().apply(config, 1.0);

    imu.setYaw(0.0, 1.0);

    imuRollSignal = imu.getRoll();
    imuPitchSignal = imu.getPitch();
    imuYawSignal = imu.getYaw();
    imuGravVectorXSignal = imu.getGravityVectorX();
    imuGravVectorYSignal = imu.getGravityVectorY();
    imuGravVectorZSignal = imu.getGravityVectorZ();
    imuAngularVelXSignal = imu.getAngularVelocityXWorld();
    imuAngularVelYSignal = imu.getAngularVelocityYWorld();
    imuAngularVelZSignal = imu.getAngularVelocityZWorld();
    imuAccelXSignal = imu.getAccelerationX();
    imuAccelYSignal = imu.getAccelerationY();
    imuAccelZSignal = imu.getAccelerationZ();
  }

  @Override
  public void updateIMUOdomInputs(IMUOdomInputs inputs) {
    inputs.measurementTimestamp = Logger.getRealTimestamp() / 1000000.0;
    inputs.rollDeg = imuRollSignal.getValue();
    inputs.pitchDeg = imuPitchSignal.getValue();
    inputs.yawDeg = imuYawSignal.getValue();
    inputs.angularVelX = imuAngularVelXSignal.getValue();
    inputs.angularVelY = imuAngularVelYSignal.getValue();
    inputs.angularVelZ = imuAngularVelZSignal.getValue();
  }

  @Override
  public void updateIMUMiscInputs(IMUMiscInputs inputs) {
    inputs.gravVectorX = imuGravVectorXSignal.getValue();
    inputs.gravVectorY = imuGravVectorYSignal.getValue();
    inputs.gravVectorZ = imuGravVectorZSignal.getValue();
    inputs.accelX = imuAccelXSignal.getValue() - inputs.gravVectorX;
    inputs.accelY = imuAccelYSignal.getValue() - inputs.gravVectorY;
    inputs.accelZ = imuAccelZSignal.getValue() - inputs.gravVectorZ;

    BaseStatusSignal.refreshAll(
        imuGravVectorXSignal,
        imuGravVectorYSignal,
        imuGravVectorZSignal,
        imuAccelXSignal,
        imuAccelYSignal,
        imuAccelZSignal);
  }

  @Override
  public BaseStatusSignal[] getOdomSignals() {
    return new BaseStatusSignal[] {
      imuRollSignal,
      imuPitchSignal,
      imuYawSignal,
      imuAngularVelXSignal,
      imuAngularVelYSignal,
      imuAngularVelZSignal
    };
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {
    subsystem.registerHardware("IMU", imu);
  }
}
