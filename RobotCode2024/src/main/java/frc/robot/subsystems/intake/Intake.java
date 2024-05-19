package frc.robot.subsystems.intake;

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
import frc.robot.RobotContainer;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class Intake extends AdvancedSubsystem {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  public static boolean hasGamePieceSimOverride = false;
  private double targetRPM = 0.0;
  private double targetSurfaceSpeed = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
    this.inputs = new IntakeInputsAutoLogged();

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, null, null, state -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism(
                (Measure<Voltage> volts) -> setVoltage(volts.in(Volts)), null, this));

    io.registerSelfCheckHardware(this);
  }

  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/TargetRPM", targetRPM);
    Logger.recordOutput("Intake/TargetSurfaceVel", targetSurfaceSpeed);
    Logger.recordOutput("Intake/RPM", getRPM());
    Logger.recordOutput("Intake/SurfaceVel", getSurfaceSpeedMps());

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Intake/PeriodicRuntimeMS", runtimeMS);
  }

  public void setTargetRpm(double rpm) {
    targetRPM = rpm;
    targetSurfaceSpeed = 0.0;
    io.setTargetMotorVel((targetRPM / 60.0) * Constants.Intake.gearing);
  }

  public void setTargetSurfaceSpeed(double surfaceSpeedMps) {
    targetSurfaceSpeed = surfaceSpeedMps;
    targetRPM = 0.0;
    io.setTargetMotorVel(
        (targetSurfaceSpeed / Constants.Intake.rollerCircumference) * Constants.Intake.gearing);
  }

  public void setVoltage(double voltage) {
    targetRPM = 0.0;
    targetSurfaceSpeed = 0.0;
    io.setVoltage(voltage);
  }

  public double getRPM() {
    return (inputs.intakeMotorVel / Constants.Intake.gearing) * 60.0;
  }

  public double getSurfaceSpeedMps() {
    return (inputs.intakeMotorVel / Constants.Intake.gearing)
        * Constants.Intake.rollerCircumference;
  }

  public double getCurrentDraw() {
    return inputs.intakeMotorSupplyCurrent;
  }

  public Command runIntakeWithPurge() {
    return run(
        () -> {
          if (RobotContainer.driver.getBButton()) {
            setVoltage(-16.0);
          } else {
            setVoltage(16.0);
          }
        });
  }

  public Command setVoltageCommand(double volt) {
    return run(() -> setVoltage(volt));
  }

  public Command runIntake() {
    return run(() -> setVoltage(16.0));
  }

  public Command reverseIntake() {
    return run(() -> setVoltage(-7.0));
  }

  public Command stopIntakeCommand() {
    return runOnce(() -> setVoltage(0.0));
  }

  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/Intake");
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
                SignalLogger.setPath("/U/sysID/Intake");
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
            run(() -> setVoltage(8.0)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getRPM() < 1000) {
                    addFault("[System Check] Intake RPM measured too low", false, true);
                  }
                }),
            run(() -> setTargetRpm(1000)).withTimeout(1.0),
            runOnce(
                () -> {
                  if (getRPM() < 800 || getRPM() > 1200) {
                    addFault("[System Check] Intake did not reach target RPM", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(() -> setVoltage(0)));
  }
}
