package frc.robot.subsystems.uptake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.util.NoteSimulator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Uptake extends AdvancedSubsystem {
  private final UptakeIO io;
  private final UptakeInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private double targetRPM = 0.0;
  private double targetSurfaceVel = 0.0;
  private double targetPos = 0.0;

  public static boolean hasGamePieceSimOverride = false;

  private boolean hasGamePieceDebounced = false;

  public Uptake(UptakeIO io) {
    this.io = io;
    this.inputs = new UptakeInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setVoltage(volts.in(Volts)), null, this));

    io.registerSelfCheckHardware(this);

    new Trigger(this::hasGamePiece)
        .debounce(0.05, Debouncer.DebounceType.kBoth)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      hasGamePieceDebounced = true;
                      NoteSimulator.attachToShooter();
                    })
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      hasGamePieceDebounced = false;
                      NoteSimulator.launch(
                          RobotContainer.shooter.getTopSurfaceSpeed()
                              / (Robot.getScoringMode() == Robot.ScoringMode.Pass
                                  ? Constants.Shooter.passSurfaceSpeedMultiplier
                                  : Constants.Shooter.surfaceSpeedMultiplier));
                    })
                .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Uptake", inputs);

    Logger.recordOutput("Uptake/HasGamePiece", hasGamePiece());
    Logger.recordOutput("Uptake/TargetRPM", targetRPM);
    Logger.recordOutput("Uptake/HasGamePiece", hasGamePiece());
    Logger.recordOutput("Uptake/HasGamePieceDebounced", hasGamePieceDebounced());
    Logger.recordOutput("Uptake/TargetSurfaceVel", targetSurfaceVel);
    Logger.recordOutput("Uptake/TargetRollerPos", targetPos);
    Logger.recordOutput("Uptake/RPM", getRPM());
    Logger.recordOutput("Uptake/SurfaceVel", getSurfaceVelMps());
    Logger.recordOutput("Uptake/RollerPos", getPositionRoller());

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Uptake/PeriodicRuntimeMS", runtimeMS);
  }

  public void setTargetRPM(double rpm) {
    targetRPM = rpm;
    targetSurfaceVel = 0.0;
    targetPos = 0.0;
    io.setTargetMotorVel((targetRPM / 60.0) * Constants.Uptake.gearing);
  }

  public void setTargetSurfaceVelocity(double mps) {
    targetRPM = 0.0;
    targetSurfaceVel = mps;
    targetPos = 0.0;
    io.setTargetMotorVel((mps / Constants.Uptake.rollerCircumference) * Constants.Uptake.gearing);
  }

  public void setTargetPos(double targetPosRoller) {
    targetRPM = 0.0;
    targetSurfaceVel = 0.0;
    targetPos = targetPosRoller;
    io.setTargetMotorPos(targetPos * Constants.Uptake.gearing);
  }

  public Command setTargetPosCommand(double targetPosRoller) {
    return run(() -> setTargetPos(targetPosRoller));
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void stopMotor() {
    targetRPM = 0;
    io.stopMotor();
  }

  public double getRPM() {
    return (inputs.motorVel / Constants.Uptake.gearing) * 60.0;
  }

  public double getSurfaceVelMps() {
    return (inputs.motorVel / Constants.Uptake.gearing) * Constants.Uptake.rollerCircumference;
  }

  public double getPositionRoller() {
    return inputs.motorPos / Constants.Uptake.gearing;
  }

  public boolean hasGamePiece() {
    if (Robot.isSimulation() && !Logger.hasReplaySource()) {
      return hasGamePieceSimOverride;
    }

    return inputs.ringSensor;
  }

  public boolean hasGamePieceDebounced() {
    return hasGamePieceDebounced;
  }

  public Command uptakeUntilHasRing() {
    return Commands.either(
        setRPMCommand(Constants.Uptake.uptakeIntakeRPM, true).until(this::hasGamePiece),
        setRPMCommand(Constants.Uptake.uptakeIntakeRPM, true).until(this::hasGamePieceDebounced),
        DriverStation::isAutonomousEnabled);
  }

  public Command setRPMCommand(double rpm, boolean limit) {
    return Commands.either(
            runOnce(() -> io.configForwardLimit(true)),
            runOnce(() -> io.configForwardLimit(false)),
            () -> limit)
        .andThen(run(() -> setTargetRPM(rpm)));
  }

  public Command setTargetSurfaceVelocityCommand(double mps) {
    return run(() -> setTargetSurfaceVelocity(mps));
  }

  public Command holdPositionCommand() {
    return defer(() -> setTargetPosCommand(getPositionRoller()));
  }

  public Command stopUptakeCommand() {
    return runOnce(this::stopMotor);
  }

  public Command shoot() {
    return setRPMCommand(Constants.Uptake.shootRPM, false)
        .alongWith(
            Commands.runOnce(
                () ->
                    NoteSimulator.launch(
                        RobotContainer.shooter.getTopSurfaceSpeed()
                            / (Robot.getScoringMode() == Robot.ScoringMode.Pass
                                ? Constants.Shooter.passSurfaceSpeedMultiplier
                                : Constants.Shooter.surfaceSpeedMultiplier))));
  }

  public Command shootUntilNoRing() {
    return shoot().until(() -> !hasGamePieceDebounced()).andThen(stopUptakeCommand());
  }

  public Command setVoltageCommand(double voltage) {
    return run(() -> setVoltage(voltage));
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/Uptake");
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
                SignalLogger.setPath("/U/sysID/Uptake");
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
            run(() -> setVoltage(-8.0)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getRPM() > -500) {
                    addFault("[System Check] Uptake RPM measured too low", false, true);
                  }
                }),
            run(() -> setTargetRPM(-700)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getRPM() > -600 || getRPM() < -800) {
                    addFault("[System Check] Uptake did not reach target RPM", false, true);
                  }
                }),
            uptakeUntilHasRing(),
            setRPMCommand(-500, false).withTimeout(0.5))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::stopUptakeCommand));
  }
}
