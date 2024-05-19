package frc.robot.subsystems.arm_extension;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class ArmExtension extends AdvancedSubsystem {
  private final ArmExtensionIO io;
  private final ArmExtensionInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private double targetExtensionPos = 0.0;

  public ArmExtension(ArmExtensionIO io) {
    this.io = io;
    this.inputs = new ArmExtensionInputsAutoLogged();

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
    Logger.processInputs("ArmExtension", inputs);

    Logger.recordOutput("ArmExtension/TargetExtension", targetExtensionPos);

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("ArmExtension/PeriodicRuntimeMS", runtimeMS);
  }

  /**
   * Set the target extension position
   *
   * @param extensionMeters Target extension, in meters
   */
  public void setTargetExtension(double extensionMeters) {
    targetExtensionPos =
        MathUtil.clamp(
            extensionMeters,
            Constants.ArmExtension.reverseLimit,
            Constants.ArmExtension.forwardLimit);
    io.setTargetExtension(targetExtensionPos);
  }

  /**
   * Get the current extension position
   *
   * @return Current extension, in meters
   */
  public double getExtensionMeters() {
    return inputs.extensionMeters;
  }

  /**
   * Get the target extension position
   *
   * @return Target extension, in meters
   */
  public double getTargetExtensionMeters() {
    return targetExtensionPos;
  }

  /**
   * Get the current velocity of the extension
   *
   * @return Current velocity, in meters
   */
  public double getVelocityMps() {
    return inputs.velocityMps;
  }

  public void brakeMotor() {
    io.brakeMotor();
  }

  public Command brakeCommand() {
    return run(this::brakeMotor);
  }

  /**
   * Create a command that will continuously set the target extension position
   *
   * @param extensionMeters The target extension position, in meters
   * @return Command that sets the target extension
   */
  public Command setTargetExtensionCommand(double extensionMeters) {
    return run(() -> setTargetExtension(extensionMeters));
  }

  /**
   * Set the voltage output to the extension motor
   *
   * @param volts Voltage to output
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Create a quasistatic sysID command
   *
   * @param direction Direction to run
   * @return Quasistatic sysID command
   */
  public Command sysIDQuasistatic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ArmJoint");
              }
              SignalLogger.start();
            }),
        sysIdRoutine.quasistatic(direction));
  }

  /**
   * Create a dynamic sysID command
   *
   * @param direction Direction to run
   * @return Dynamic sysID command
   */
  public Command sysIDDynamic(SysIdRoutine.Direction direction) {
    return Commands.sequence(
        runOnce(
            () -> {
              io.optimizeForSysID();
              if (Robot.isReal()) {
                SignalLogger.setPath("/U/sysID/ArmJoint");
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
                  setVoltage(4.0);
                }),
            Commands.waitSeconds(0.5),
            runOnce(
                () -> {
                  if (getExtensionMeters() < 0.1) {
                    addFault("[System Check] Arm Extension Position measured too low", false, true);
                  }
                  if (getVelocityMps() < 0.1) {
                    addFault("[System Check] Velocity measured too low", false, true);
                  }
                }),
            setTargetExtensionCommand(0.35).withTimeout(1),
            runOnce(
                () -> {
                  if (getExtensionMeters() < 0.3 || getExtensionMeters() > 0.4) {
                    addFault(
                        "[System Check] Arm Extension did not reach target position", false, true);
                  }
                }),
            setTargetExtensionCommand(0).withTimeout(1),
            runOnce(
                () -> {
                  if (getExtensionMeters() < -0.5 || getExtensionMeters() > 0.5) {
                    addFault(
                        "[System Check] Arm Extension did not reach target position", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::brakeMotor));
  }
}
