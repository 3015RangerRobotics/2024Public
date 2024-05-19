package frc.robot.subsystems.arm_joint;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class ArmJoint extends AdvancedSubsystem {
  private final ArmJointIO io;
  private final ArmJointInputsAutoLogged inputs;

  private final SysIdRoutine sysIdRoutine;

  private Rotation2d targetRotation = Constants.ArmJoint.reverseLimit;

  public ArmJoint(ArmJointIO io) {
    this.io = io;
    this.inputs = new ArmJointInputsAutoLogged();

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
    Logger.processInputs("ArmJoint", inputs);

    Logger.recordOutput("ArmJoint/TargetAngle", targetRotation);

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("ArmJoint/PeriodicRuntimeMS", runtimeMS);
  }

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  public void setTargetAngle(Rotation2d angle) {
    targetRotation = angle;
    io.setTargetAngle(targetRotation);
  }

  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {
    targetRotation = angle;
    io.setTargetAngle(targetRotation, maxVel, maxAccel);
  }

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  /**
   * Get the current angle of the arm joint
   *
   * @return Current angle
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(inputs.jointAngleDegrees);
  }

  /**
   * Get the current velocity of the arm joint
   *
   * @return Velocity, in degrees per seconds
   */
  public double getVelocityDps() {
    return inputs.jointVelocityDps;
  }

  /**
   * Get the target arm joint angle
   *
   * @return Target angle
   */
  public Rotation2d getTargetAngle() {
    return targetRotation;
  }

  public void brakeMotors() {
    io.brakeMotors();
  }

  public Command setTargetAngleCommand(Rotation2d angle) {
    return run(() -> setTargetAngle(angle));
  }

  public Command setTargetAngleCommand(Rotation2d angle, double maxVel, double maxAccel) {
    return run(() -> setTargetAngle(angle, maxVel, maxAccel));
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
                SignalLogger.setPath("/U/sysID/ArmJoint");
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
                  setVoltage(3.0);
                }),
            Commands.waitSeconds(0.5),
            runOnce(
                () -> {
                  if (getVelocityDps() < 10) {
                    addFault("[System Check] Velocity measured too low", false, true);
                  }
                }),
            setTargetAngleCommand(Rotation2d.fromDegrees(90)).withTimeout(1),
            runOnce(
                () -> {
                  if (inputs.jointAngleDegrees < 85 || inputs.jointAngleDegrees > 95) {
                    addFault("[System Check] Arm Joint did not reach target position", false, true);
                  }
                }),
            setTargetAngleCommand(Constants.ArmJoint.restPositionAngle).withTimeout(1),
            runOnce(
                () -> {
                  if (inputs.jointAngleDegrees
                          < Constants.ArmJoint.restPositionAngle.getDegrees() - 5
                      || inputs.jointAngleDegrees
                          > Constants.ArmJoint.restPositionAngle.getDegrees() + 5) {
                    addFault("[System Check] Arm Joint did not reach target position", false, true);
                  }
                }))
        .until(() -> !getFaults().isEmpty())
        .andThen(runOnce(this::brakeMotors));
  }
}
