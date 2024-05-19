package frc.robot.subsystems.swerve_module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import java.util.Collections;
import java.util.List;

public class SwerveModuleIOSim implements SwerveModuleIO {
  private final LinearSystemSim<N1, N1, N1> driveSim;
  private final LinearSystemSim<N2, N1, N1> rotationSim;

  private final PIDController driveVelController;
  private final PIDController rotationPosController;

  private boolean driveClosedLoop;
  private double driveFFVolts;
  private double driveAppliedVolts;
  private boolean rotationClosedLoop;
  private double rotationAppliedVolts;

  private double lastDrivePos = 0.0;
  private double lastRotationPos = 0.0;

  public SwerveModuleIOSim() {
    driveSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyVelocitySystem(
                Constants.SwerveModule.drivekV, Constants.SwerveModule.drivekA));
    rotationSim =
        new LinearSystemSim<>(
            LinearSystemId.identifyPositionSystem(Constants.SwerveModule.rotationkV, 0.0001));

    driveVelController =
        new PIDController(
            Constants.SwerveModule.drivekP,
            Constants.SwerveModule.drivekI,
            Constants.SwerveModule.drivekD);
    rotationPosController =
        new PIDController(
            Constants.SwerveModule.rotationkP,
            Constants.SwerveModule.rotationkI,
            Constants.SwerveModule.rotationkD);
    rotationPosController.enableContinuousInput(-0.5, 0.5);

    driveClosedLoop = false;
    driveFFVolts = 0.0;
    driveAppliedVolts = 0.0;
    rotationClosedLoop = false;
    rotationAppliedVolts = 0.0;
  }

  /**
   * Update the odometry inputs. This should not refresh any signals, as that will be handled in the
   * odometry thread
   *
   * @param inputs
   */
  @Override
  public void updateOdometryInputs(SwerveModuleOdometryInputs inputs) {
    double dt = 1.0 / Constants.Swerve.odomUpdateHz;

    if (driveClosedLoop) {
      double currentVelRot = driveSim.getOutput(0);
      driveAppliedVolts =
          MathUtil.clamp(driveVelController.calculate(currentVelRot) + driveFFVolts, -12.0, 12.0);
    }
    if (rotationClosedLoop) {
      double currentPosRot = rotationSim.getOutput(0);
      rotationAppliedVolts =
          MathUtil.clamp(rotationPosController.calculate(currentPosRot), -12.0, 12.0);
    }

    driveSim.setInput(driveAppliedVolts);
    rotationSim.setInput(rotationAppliedVolts);

    driveSim.update(dt);
    rotationSim.update(dt);

    double driveVelRot = driveSim.getOutput(0);
    double drivePosRot = lastDrivePos + (driveVelRot * dt);
    lastDrivePos = drivePosRot;

    double rotationPosRot = rotationSim.getOutput(0);
    double rotationDeltaPosRot = rotationPosRot - lastRotationPos;
    double rotationVelRot = rotationDeltaPosRot / dt;
    lastRotationPos = rotationPosRot;

    inputs.drivePositionRot = drivePosRot;
    inputs.driveVelocityRps = driveVelRot;

    inputs.rotationPositionDegrees =
        Units.rotationsToDegrees(MathUtil.inputModulus(rotationPosRot, -0.5, 0.5));
    inputs.rotationVelocityDps = Units.rotationsToDegrees(rotationVelRot);
  }

  /**
   * Update the misc inputs. This should refresh signals.
   *
   * @param inputs
   */
  @Override
  public void updateMiscInputs(SwerveModuleMiscInputs inputs) {
    inputs.rotationTemperature = 0.0;
    inputs.driveTemperature = 0.0;
  }

  @Override
  public void setTargetDriveVelocity(double driveVelocityMps) {
    driveClosedLoop = true;
    driveFFVolts =
        (driveVelocityMps * Constants.SwerveModule.driveRotationsPerMeter)
            * Constants.SwerveModule.drivekV;
    driveVelController.setSetpoint(
        driveVelocityMps * Constants.SwerveModule.driveRotationsPerMeter);
  }

  @Override
  public void setTargetRotation(double rotationDegrees) {
    rotationClosedLoop = true;
    rotationPosController.setSetpoint(Units.degreesToRotations(rotationDegrees));
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveClosedLoop = false;
    driveAppliedVolts = volts;
  }

  @Override
  public void setRotationVoltage(double volts) {
    rotationClosedLoop = false;
    rotationAppliedVolts = volts;
  }

  @Override
  public void stopMotors() {
    brakeDrive();
    brakeRotation();
  }

  @Override
  public void brakeDrive() {
    driveClosedLoop = false;
    driveAppliedVolts = 0.0;
  }

  @Override
  public void brakeRotation() {
    rotationClosedLoop = false;
    rotationAppliedVolts = 0.0;
  }

  @Override
  public void registerSelfCheckHardware(AdvancedSubsystem subsystem) {}

  @Override
  public BaseStatusSignal[] getOdometrySignals() {
    return new BaseStatusSignal[0];
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }
}
