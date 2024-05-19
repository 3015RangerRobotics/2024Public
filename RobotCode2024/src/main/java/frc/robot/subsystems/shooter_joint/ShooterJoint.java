package frc.robot.subsystems.shooter_joint;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.LaunchCalculator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ShooterJoint extends AdvancedSubsystem {
  private final ShooterJointIO io;
  private final ShooterJointInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private Rotation2d targetAngle = new Rotation2d();

  public ShooterJoint(ShooterJointIO io) {
    this.io = io;
    this.inputs = new ShooterJointInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setVoltage(volts.in(Volts)), null, this));

    io.registerSelfCheckHardware(this);
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("ShooterJoint", inputs);

    Logger.recordOutput("ShooterJoint/TargetAngle", targetAngle);
    Logger.recordOutput("ShooterJoint/Angle", getAngle());

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("ShooterJoint/PeriodicRuntimeMS", runtimeMS);
  }

  public Command targetLaunchAngle() {
    return run(
        () -> {
          LaunchCalculator.LaunchState currentLaunch = LaunchCalculator.getCurrentLaunchState();

          if (!currentLaunch.valid()) {
            setTargetAngleProfiled(Rotation2d.fromDegrees(20));
            return;
          }

          Rotation2d launchAngle = new Rotation2d(currentLaunch.launchAngle().getY());

          if (RobotContainer.shouldShootReversed()) {
            setTargetAngle(
                Rotation2d.fromDegrees(-90)
                    .minus(RobotContainer.armJoint.getAngle())
                    .plus(launchAngle));
          } else {
            Rotation2d armOffset =
                Rotation2d.fromDegrees(90).minus(RobotContainer.armJoint.getAngle());
            setTargetAngle(armOffset.minus(launchAngle));
          }
        });
  }

  public void setTargetAngle(Rotation2d angle) {
    targetAngle = angle;
    io.setTargetAngle(targetAngle);
  }

  public void setTargetAngleProfiled(Rotation2d angle) {
    targetAngle = angle;
    io.setTargetAngleProfiled(targetAngle);
  }

  public void setVoltage(double voltage) {
    io.setVoltage(voltage);
  }

  public Command setTargetAngleCommand(Rotation2d angle) {
    return run(() -> setTargetAngle(angle));
  }

  public Command setTargetAngleProfiledCommand(Rotation2d angle) {
    return run(() -> setTargetAngleProfiled(angle));
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(inputs.jointAngleDegrees);
  }

  public Rotation2d getTargetAngle() {
    return targetAngle;
  }

  public double getVelocityDps() {
    return inputs.jointVelocityDps;
  }

  public double getTotalCurrentDraw() {
    return inputs.jointMotorSupplyCurrent;
  }

  public void brakeMotor() {
    io.brakeMotor();
  }

  public Command brakeCommand() {
    return run(this::brakeMotor);
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ShooterJoint");
              }
              SignalLogger.start();
            }),
        sysIdRoutine.quasistatic(direction));
  }

  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ShooterJoint");
              }
              SignalLogger.start();
            }),
        sysIdRoutine.dynamic(direction));
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
                  setVoltage(2.0);
                }),
            Commands.waitSeconds(0.3),
            runOnce(
                () -> {
                  if (getVelocityDps() < 10) {
                    addFault("[System Check] Velocity measured too low", false, true);
                  }
                }),
            setTargetAngleProfiledCommand(Rotation2d.fromDegrees(45)).withTimeout(1),
            runOnce(
                () -> {
                  if (inputs.jointAngleDegrees < 40 || inputs.jointAngleDegrees > 50) {
                    addFault(
                        "[System Check] Shooter Joint did not reach target position", false, true);
                  }
                }),
            setTargetAngleProfiledCommand(Rotation2d.fromDegrees(0)).withTimeout(1),
            runOnce(
                () -> {
                  if (inputs.jointAngleDegrees < -5 || inputs.jointAngleDegrees > 5) {
                    addFault(
                        "[System Check] Shooter Joint Angle is did not reach target position",
                        false,
                        true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::brakeMotor));
  }
}
