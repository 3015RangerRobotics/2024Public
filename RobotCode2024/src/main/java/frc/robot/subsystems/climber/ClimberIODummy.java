package frc.robot.subsystems.climber;

public class ClimberIODummy implements ClimberIO {
  public ClimberIODummy() {}

  /**
   * Update the inputs for the climber
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ClimberInputs inputs) {}

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  @Override
  public void setLeftClimberLength(double length) {}

  /**
   * Set the target length of the left climber
   *
   * @param length Target length, in meters
   */
  @Override
  public void setRightClimberLength(double length) {}

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setLeftVoltage(double volts) {}

  /**
   * Set the voltage output to the left climber motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setRightVoltage(double volts) {}

  @Override
  public void brakeMotors() {}
}
