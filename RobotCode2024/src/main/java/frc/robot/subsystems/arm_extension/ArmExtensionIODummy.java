package frc.robot.subsystems.arm_extension;

public class ArmExtensionIODummy implements ArmExtensionIO {
  /**
   * Create a dummy IO implementation for the arm extension. This is useful for disabling this
   * system on a real robot
   */
  public ArmExtensionIODummy() {}

  /**
   * Update the inputs for the arm extension
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmExtensionInputs inputs) {}

  /**
   * Set the target extension length
   *
   * @param lengthMeters Target extension length, in meters
   */
  @Override
  public void setTargetExtension(double lengthMeters) {}

  /**
   * Output a set voltage to the arm extension motor
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {}

  @Override
  public void brakeMotor() {}
}
