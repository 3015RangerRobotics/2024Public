package frc.robot.subsystems.uptake;

public class UptakeIODummy implements UptakeIO {
  public UptakeIODummy() {}

  @Override
  public void updateInputs(UptakeInputs inputs) {}

  @Override
  public void setTargetMotorVel(double motorVel) {}

  @Override
  public void setTargetMotorPos(double motorPos) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void stopMotor() {}
}
