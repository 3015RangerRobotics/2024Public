package frc.robot.subsystems.shooter;

public class ShooterIODummy implements ShooterIO {
  public ShooterIODummy() {}

  @Override
  public void updateInputs(ShooterInputs inputs) {}

  @Override
  public void setTargetMotorVel(double topMotorVel, double bottomMotorVel) {}

  @Override
  public void setVoltage(double topVolts, double bottomVolts) {}

  @Override
  public void stopMotors() {}
}
