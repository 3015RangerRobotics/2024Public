package frc.robot.subsystems.swerve_module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends AdvancedSubsystem {
  public enum ModuleCode {
    FL,
    FR,
    BL,
    BR
  }

  public final ModuleCode moduleCode;
  private SwerveModuleState targetState = new SwerveModuleState();

  private final SwerveModuleIO io;
  private final SwerveModuleOdometryInputsAutoLogged odometryInputs =
      new SwerveModuleOdometryInputsAutoLogged();
  private final SwerveModuleMiscInputsAutoLogged miscInputs =
      new SwerveModuleMiscInputsAutoLogged();

  public SwerveModule(ModuleCode moduleCode, SwerveModuleIO io) {
    super(moduleCode.name() + "SwerveModule");

    this.moduleCode = moduleCode;
    this.io = io;

    this.io.registerSelfCheckHardware(this);
  }

  @Override
  public void periodic() {
    io.updateMiscInputs(miscInputs);

    Logger.processInputs("Swerve/" + getName() + "/Misc", miscInputs);
  }

  public void updateOdometryInputs() {
    io.updateOdometryInputs(odometryInputs);

    Logger.processInputs("Swerve/" + getName() + "/Odom", odometryInputs);
  }

  public void setTargetState(SwerveModuleState targetState) {
    SwerveModuleState currentState = getState();

    SwerveModuleState optimized = SwerveModuleState.optimize(targetState, currentState.angle);

    if (Math.abs(optimized.speedMetersPerSecond) < 0.02) {
      stopMotors();
      return;
    }

    Rotation2d rotationError = optimized.angle.minus(currentState.angle);
    double velocityTarget = optimized.speedMetersPerSecond * rotationError.getCos();

    this.targetState = new SwerveModuleState(velocityTarget, optimized.angle);

    io.setTargetDriveVelocity(velocityTarget);
    io.setTargetRotation(this.targetState.angle.getDegrees());
  }

  public void stopMotors() {
    io.stopMotors();
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
        odometryInputs.driveVelocityRps / Constants.SwerveModule.driveRotationsPerMeter,
        Rotation2d.fromDegrees(odometryInputs.rotationPositionDegrees));
  }

  public SwerveModuleState getTargetState() {
    return targetState;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        odometryInputs.drivePositionRot / Constants.SwerveModule.driveRotationsPerMeter,
        Rotation2d.fromDegrees(odometryInputs.rotationPositionDegrees));
  }

  public void lockModule() {
    double targetAngle = -45;
    if (moduleCode == ModuleCode.FL || moduleCode == ModuleCode.BR) {
      targetAngle = 45;
    }

    targetState = new SwerveModuleState(0, Rotation2d.fromDegrees(targetAngle));
    targetState = SwerveModuleState.optimize(targetState, getPosition().angle);

    io.brakeDrive();
    io.setTargetRotation(targetState.angle.getDegrees());
  }

  public void brakeDrive() {
    io.brakeDrive();
  }

  public void brakeRotation() {
    io.brakeRotation();
  }

  public void brakeMotors() {
    brakeDrive();
    brakeRotation();
  }

  public void setTargetRotation(double degrees) {
    io.setTargetRotation(degrees);
  }

  public void setDriveVoltage(double volts) {
    io.setDriveVoltage(volts);
  }

  public void setRotationVoltage(double volts) {
    io.setRotationVoltage(volts);
  }

  public BaseStatusSignal[] getOdomSignals() {
    return io.getOdometrySignals();
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return io.getOrchestraDevices();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.sequence(
            runOnce(
                () -> {
                  clearFaults();
                  io.brakeDrive();
                  io.setRotationVoltage(3.6);
                }),
            Commands.waitSeconds(0.3),
            runOnce(
                () -> {
                  if (odometryInputs.rotationVelocityDps < 20) {
                    addFault(
                        "[System Check] Rotation encoder velocity measured too slow", false, true);
                  }
                }),
            run(() -> io.setTargetRotation(90.0)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getState().angle.getDegrees() < 70 || getState().angle.getDegrees() > 110) {
                    addFault(
                        "[System Check] Rotation Motor did not reach target position", false, true);
                  }
                }),
            runOnce(
                () -> {
                  io.setDriveVoltage(1.2);
                  io.brakeRotation();
                }),
            Commands.waitSeconds(0.5),
            runOnce(
                () -> {
                  if (getState().speedMetersPerSecond < 0.25) {
                    addFault("[System Check] Drive motor encoder velocity too slow", false, true);
                  }
                  io.brakeDrive();
                }),
            Commands.waitSeconds(0.25),
            run(() -> io.setTargetRotation(0.0)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (Math.abs(getState().angle.getDegrees()) > 20) {
                    addFault("[System Check] Rotation did not reach target position", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::stopMotors));
  }
}
