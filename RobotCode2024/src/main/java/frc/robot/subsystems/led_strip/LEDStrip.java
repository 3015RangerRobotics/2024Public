package frc.robot.subsystems.led_strip;

import com.ctre.phoenix.led.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.auto.NoneAuto;
import org.littletonrobotics.junction.Logger;

public class LEDStrip extends SubsystemBase {
  public enum State {
    PARTY,
    NORMAL,
    AUTO,
    SAD
  }

  private final LEDStripIO io;
  private State state = State.NORMAL;

  public LEDStrip(LEDStripIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    double startTime = Timer.getFPGATimestamp();

    if (DriverStation.isDisabled()) {
      io.setStatusLED(RobotContainer.swerve.getSystemStatus(), Constants.LEDStrip.swerveLED);
      io.setStatusLED(RobotContainer.armJoint.getSystemStatus(), Constants.LEDStrip.armJointLED);
      io.setStatusLED(
          RobotContainer.armExtension.getSystemStatus(), Constants.LEDStrip.armExtensionLED);
      io.setStatusLED(
          RobotContainer.shooterJoint.getSystemStatus(), Constants.LEDStrip.shooterJointLED);
      io.setStatusLED(RobotContainer.shooter.getSystemStatus(), Constants.LEDStrip.shooterLED);
      io.setStatusLED(RobotContainer.intake.getSystemStatus(), Constants.LEDStrip.intakeLED);
      io.setStatusLED(RobotContainer.uptake.getSystemStatus(), Constants.LEDStrip.uptakeLED);
      io.setStatusLED(RobotContainer.climber.getSystemStatus(), Constants.LEDStrip.climberLED);
    }

    switch (state) {
      case NORMAL:
        if (DriverStation.isDSAttached()) {
          if (DriverStation.isAutonomousEnabled()) {
            io.fireAnimation();
          } else {
            if (DriverStation.isDisabled()
                && RobotContainer.getAutonomousCommand() instanceof NoneAuto) {
              io.sadAnimation();
            } else {
              allianceAnim();
            }
          }
        } else {
          io.breathingWhite();
        }
        break;
      case AUTO:
        io.fireAnimation();
        break;
      case PARTY:
        io.partyAnimation();
        break;
      case SAD:
        io.sadAnimation();
        break;
    }

    double runtimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
    Logger.recordOutput("LEDStrip/PeriodicRuntime", runtimeMS);
  }

  public void setState(State state) {
    this.state = state;
  }

  public State getState() {
    return state;
  }

  public Command setStateCommand(State state) {
    return runOnce(() -> setState(state));
  }

  private void allianceAnim() {
    if (Robot.isRedAlliance()) {
      io.redAllianceAnimation();
    } else {
      io.blueAllianceAnimation();
    }
  }
}
