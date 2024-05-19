package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.LaunchCalculator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Shooter extends AdvancedSubsystem {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private double targetRPMTop = 0.0;
  private double targetSurfaceSpeedTop = 0.0;
  private double targetRPMBottom = 0.0;
  private double targetSurfaceSpeedBottom = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
    this.inputs = new ShooterInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setVoltage(volts.in(Volts), volts.in(Volts)),
                null,
                this));

    io.registerSelfCheckHardware(this);
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Shooter/TopTargetRPM", targetRPMTop);
    Logger.recordOutput("Shooter/TopTargetSurfaceSpeed", targetSurfaceSpeedTop);
    Logger.recordOutput("Shooter/TopRPM", getTopRPM());
    Logger.recordOutput("Shooter/TopSurfaceSpeed", getTopSurfaceSpeed());

    Logger.recordOutput("Shooter/BottomTargetRPM", targetRPMBottom);
    Logger.recordOutput("Shooter/BottomTargetSurfaceSpeed", targetSurfaceSpeedBottom);
    Logger.recordOutput("Shooter/BottomRPM", getBottomRPM());
    Logger.recordOutput("Shooter/BottomSurfaceSpeed", getBottomSurfaceSpeed());

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Shooter/PeriodicRuntimeMS", runtimeMS);
  }

  public void setTargetRPM(double rpm) {
    targetRPMTop = rpm;
    targetRPMBottom = rpm;
    targetSurfaceSpeedTop = 0.0;
    targetSurfaceSpeedBottom = 0.0;
    io.setTargetMotorVel(
        (targetRPMTop / 60.0) * Constants.Shooter.gearing,
        (targetRPMBottom / 60.0) * Constants.Shooter.gearing);
  }

  public void setTargetSurfaceVelocity(double mps) {
    targetRPMTop = 0.0;
    targetRPMBottom = 0.0;
    targetSurfaceSpeedTop = mps;
    targetSurfaceSpeedBottom = mps;
    io.setTargetMotorVel(
        (targetSurfaceSpeedTop / Constants.Shooter.rollerCircumference) * Constants.Shooter.gearing,
        (targetSurfaceSpeedBottom / Constants.Shooter.rollerCircumference)
            * Constants.Shooter.gearing);
  }

  public void setVoltage(double topVolts, double bottomVolts) {
    targetRPMTop = 0.0;
    targetRPMBottom = 0.0;
    targetSurfaceSpeedTop = 0.0;
    targetSurfaceSpeedBottom = 0.0;
    io.setVoltage(topVolts, bottomVolts);
  }

  public void stopMotors() {
    targetRPMTop = 0.0;
    targetRPMBottom = 0.0;
    targetSurfaceSpeedTop = 0.0;
    targetSurfaceSpeedBottom = 0.0;
    io.stopMotors();
  }

  public double getTopRPM() {
    return (inputs.topMotorVel / Constants.Shooter.gearing) * 60.0;
  }

  public double getBottomRPM() {
    return (inputs.bottomMotorVel / Constants.Shooter.gearing) * 60.0;
  }

  public double getTopSurfaceSpeed() {
    return (inputs.topMotorVel / Constants.Shooter.gearing) * Constants.Shooter.rollerCircumference;
  }

  public double getBottomSurfaceSpeed() {
    return (inputs.bottomMotorVel / Constants.Shooter.gearing)
        * Constants.Shooter.rollerCircumference;
  }

  public double getTopTargetSurfaceSpeed() {
    return targetSurfaceSpeedTop;
  }

  public double getBottomTargetSurfaceSpeed() {
    return targetSurfaceSpeedBottom;
  }

  public double getTopTargetRPM() {
    return targetRPMTop;
  }

  public double getBottomTargetRPM() {
    return targetRPMBottom;
  }

  public Command setRPMCommand(double rpm) {
    return run(() -> setTargetRPM(rpm));
  }

  public Command setTargetSurfaceVelocityCommand(double mps) {
    return run(() -> setTargetSurfaceVelocity(mps));
  }

  public Command setLaunchVelocityCommand() {
    return run(
        () ->
            setTargetSurfaceVelocity(
                LaunchCalculator.getCurrentLaunchState().launchVelocity()
                    * (Robot.getScoringMode() == Robot.ScoringMode.Pass
                        ? Constants.Shooter.passSurfaceSpeedMultiplier
                        : Constants.Shooter.surfaceSpeedMultiplier)));
  }

  public Command stopShooterCommand() {
    return run(this::stopMotors);
  }

  public Command slowShoot() {
    return setRPMCommand(Constants.Shooter.slowShootRPM);
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/Shooter");
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
                SignalLogger.setPath("/U/sysID/Shooter");
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
            runOnce(this::clearFaults),
            run(() -> setVoltage(4.0, 4.0)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getTopRPM() < 1000) {
                    addFault("[System Check] Top shooter RPM measured too low", false, true);
                  }
                  if (getBottomRPM() < 1000) {
                    addFault("[System Check] Bottom shooter RPM measured too low", false, true);
                  }
                }),
            setRPMCommand(5000).withTimeout(2),
            runOnce(
                () -> {
                  if (getTopRPM() < targetRPMTop - 300 || getTopRPM() > targetRPMTop + 300) {
                    addFault("[System Check] Top shooter did not reach target RPM", false, true);
                  }
                  if (getBottomRPM() < targetRPMBottom - 300
                      || getBottomRPM() > targetRPMBottom + 300) {
                    addFault("[System Check] Bottom shooter did not reach target RPM", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::stopMotors));
  }
}
