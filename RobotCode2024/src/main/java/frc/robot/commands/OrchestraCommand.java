package frc.robot.commands;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.List;

public class OrchestraCommand extends Command {
  private final Orchestra orchestra;

  public OrchestraCommand(String fileName) {
    List<ParentDevice> allDevices = new ArrayList<>();
    allDevices.addAll(RobotContainer.swerve.getOrchestraDevices());
    allDevices.addAll(RobotContainer.armJoint.getOrchestraDevices());
    allDevices.addAll(RobotContainer.armExtension.getOrchestraDevices());
    allDevices.addAll(RobotContainer.climber.getOrchestraDevices());
    allDevices.addAll(RobotContainer.intake.getOrchestraDevices());
    allDevices.addAll(RobotContainer.shooter.getOrchestraDevices());
    allDevices.addAll(RobotContainer.shooterJoint.getOrchestraDevices());
    allDevices.addAll(RobotContainer.uptake.getOrchestraDevices());

    orchestra = new Orchestra(allDevices, fileName + ".chrp");

    addRequirements(
        RobotContainer.swerve,
        RobotContainer.armJoint,
        RobotContainer.armExtension,
        RobotContainer.climber,
        RobotContainer.intake,
        RobotContainer.shooter,
        RobotContainer.shooterJoint,
        RobotContainer.uptake);
  }

  @Override
  public void initialize() {
    orchestra.stop();
    orchestra.play();
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return !orchestra.isPlaying();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      orchestra.stop();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
