package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.led_strip.LEDStrip;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Climber extends AdvancedSubsystem {
  private final ClimberIO io;
  private final ClimberInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private double targetLeftLength = 0.0;
  private double targetRightLength = 0.0;

  public Climber(ClimberIO io) {
    this.io = io;
    this.inputs = new ClimberInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setVoltage(volts.in(Volts), volts.in(Volts)),
                null,
                this));

    io.registerSelfCheckHardware(this);

    SmartDashboard.putData("climb go down", setVoltageCommand(-2.0));
  }

  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);

    Logger.recordOutput("Climber/TargetLengthLeft", targetLeftLength);
    Logger.recordOutput("Climber/TargetLengthRight", targetRightLength);

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Climber/PeriodicRuntimeMS", runtimeMS);
  }

  public void setClimberLength(double leftLength, double rightLength) {
    targetLeftLength = leftLength;
    targetRightLength = rightLength;

    io.setLeftClimberLength(targetLeftLength);
    io.setRightClimberLength(targetRightLength);
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    io.setLeftVoltage(leftVolts);
    io.setRightVoltage(rightVolts);
  }

  public Command setVoltageCommand(double volts) {
    return run(() -> setVoltage(volts, volts));
  }

  public double getClimberLeftLength() {
    return inputs.leftClimberLengthMeters;
  }

  public double getClimberRightLength() {
    return inputs.rightClimberLengthMeters;
  }

  public double getClimberLeftVelocityMps() {
    return inputs.leftClimberVelocityMps;
  }

  public double getClimberRightVelocityMps() {
    return inputs.rightClimberVelocityMps;
  }

  public void brakeMotors() {
    io.brakeMotors();
  }

  public Command setTargetLengthCommand(double leftLengthMeters, double rightLengthMeters) {
    return run(() -> setClimberLength(leftLengthMeters, rightLengthMeters));
  }

  public Command climberExtend() {
    return setTargetLengthCommand(
        Constants.Climber.climberPosExtend, Constants.Climber.climberPosExtend);
  }

  public Command climberRetract() {
    return setTargetLengthCommand(
        Constants.Climber.climberPosRetract, Constants.Climber.climberPosRetract);
  }

  public Command resetClimbers() {
    return setVoltageCommand(-2.5)
        .withTimeout(0.25)
        .andThen(
            setVoltageCommand(-2.5)
                .until(
                    () ->
                        Math.abs(getClimberLeftVelocityMps()) < 0.05
                            && Math.abs(getClimberRightVelocityMps()) < 0.05),
            runOnce(
                () -> {
                  setVoltage(0, 0);
                  io.resetPosition();
                }));
  }

  public Command safetyClimber() {
    return setTargetLengthCommand(
            Constants.Climber.climberPosSafety, Constants.Climber.climberPosSafety)
        .withTimeout(0.3)
        .andThen(
            Commands.either(
                Commands.none(),
                Commands.run(() -> RobotContainer.ledStrip.setState(LEDStrip.State.SAD))
                    .alongWith(climberExtend()),
                () ->
                    inputs.leftClimberStatorCurrent >= Constants.Climber.climberCurrentCheck
                        && inputs.rightClimberStatorCurrent
                            >= Constants.Climber.climberCurrentCheck));
  }

  public Command brakeCommand() {
    return run(this::brakeMotors);
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/Climber");
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
                SignalLogger.setPath("/U/sysID/Climber");
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
            resetClimbers(),
            runOnce(
                () -> {
                  clearFaults();
                  setVoltage(3.0, 3.0);
                }),
            Commands.waitSeconds(0.5),
            runOnce(
                () -> {
                  if (getClimberLeftLength() < 0.1) {
                    addFault("[System Check] Left Climber Position measured too low", false, true);
                  }
                  if (getClimberRightLength() < 0.1) {
                    addFault("[System Check] Right Climber Position measured too low", false, true);
                  }
                  if (getClimberLeftVelocityMps() < 0.1) {
                    addFault("[System Check] Left Climber Velocity measured too low", false, true);
                  }
                  if (getClimberRightVelocityMps() < 0.1) {
                    addFault("[System Check] Right Climber Velocity measured too low", false, true);
                  }
                }),
            setTargetLengthCommand(0.35, 0.35).withTimeout(1),
            runOnce(
                () -> {
                  if (getClimberLeftLength() < 0.3 || getClimberLeftLength() > 0.4) {
                    addFault(
                        "[System Check] Left Climber did not reach target position", false, true);
                  }
                  if (getClimberRightLength() < 0.3 || getClimberRightLength() > 0.4) {
                    addFault(
                        "[System Check] Right Climber did not reach target position", false, true);
                  }
                }),
            setTargetLengthCommand(0, 0).withTimeout(1),
            runOnce(
                () -> {
                  if (getClimberLeftLength() < -0.5 || getClimberLeftLength() > 0.5) {
                    addFault(
                        "[System Check] Left Climber did not reach target position", false, true);
                  }
                  if (getClimberRightLength() < -0.5 || getClimberRightLength() > 0.5) {
                    addFault(
                        "[System Check] Right Climber did not reach target position", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::brakeMotors));
  }
}
