package frc.robot.subsystems.limelight_notes;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumbleOn;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LimelightNotes extends AdvancedSubsystem {
  private final LimelightNotesIO io;
  private final LimelightNotesInputsAutoLogged inputs;

  public LimelightNotes(LimelightNotesIO io) {
    this.io = io;
    this.inputs = new LimelightNotesInputsAutoLogged();

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && !RobotContainer.uptake.hasGamePieceDebounced()
                    && hasTarget())
        .onTrue(
            Commands.runOnce(() -> RobotContainer.driver.setRumbleAnimation(new RumbleOn(0.75)))
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(() -> RobotContainer.driver.setRumbleAnimation(new RumbleOff()))
                .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    if (!Logger.hasReplaySource()) {
      io.updateInputs(inputs);
    }

    Logger.processInputs("LimelightNotes", inputs);

    if (hasTarget()) {
      Logger.recordOutput(
          "LimelightNotes/EstimatedFieldPos", new Pose3d(getEstimatedFieldPos(), new Rotation3d()));
    } else {
      Logger.recordOutput("LimelightNotes/EstimatedFieldPos", new Pose3d());
    }

    if (Timer.getFPGATimestamp() > 30.0
        && Timer.getFPGATimestamp() - inputs.lastFPSTimestamp > 3.0) {
      addFault("Limelight disconnected");
    }

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("LimelightNotes/PeriodicRuntimeMS", runtimeMS);
  }

  public boolean hasTarget() {
    return inputs.hasTarget && (Timer.getFPGATimestamp() - inputs.lastFPSTimestamp < 5.0);
  }

  public Rotation2d getAngleToTarget() {
    return Rotation2d.fromDegrees(-inputs.tx);
  }

  public Rotation2d getTY() {
    return Rotation2d.fromDegrees(inputs.ty);
  }

  public double getMeasurementTimestamp() {
    return inputs.timestamp;
  }

  public Translation3d getEstimatedFieldPos() {
    Pose3d robotPose =
        new Pose3d(RobotContainer.swerve.getPoseAtTimestamp(inputs.timestamp).orElse(new Pose2d()));
    Pose3d llPose =
        robotPose.transformBy(
            new Transform3d(
                Robot.currentZEDPoseRobot.getTranslation(),
                Robot.currentZEDPoseRobot.getRotation()));
    Logger.recordOutput("LLPose", llPose);

    double estDistance =
        Math.abs(
            (llPose.getZ() - 0.025)
                / Math.sin(Units.degreesToRadians(inputs.ty) - llPose.getRotation().getY()));
    Translation3d llRelative =
        new Translation3d(
            estDistance,
            new Rotation3d(
                0.0, Units.degreesToRadians(-inputs.ty), Units.degreesToRadians(-inputs.tx)));

    Translation3d fieldRelative =
        llRelative.rotateBy(llPose.getRotation()).plus(llPose.getTranslation());
    return new Translation3d(fieldRelative.getX(), fieldRelative.getY(), 0.025);
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }
}
