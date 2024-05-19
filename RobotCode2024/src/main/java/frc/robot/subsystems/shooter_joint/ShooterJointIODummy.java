package frc.robot.subsystems.shooter_joint;

import edu.wpi.first.math.geometry.Rotation2d;

public class ShooterJointIODummy implements ShooterJointIO {
  public ShooterJointIODummy() {}

  @Override
  public void updateInputs(ShooterJointInputs inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void brakeMotor() {}

  @Override
  public void setTargetAngle(Rotation2d angle) {}

  @Override
  public void setTargetAngleProfiled(Rotation2d angle) {}
}
