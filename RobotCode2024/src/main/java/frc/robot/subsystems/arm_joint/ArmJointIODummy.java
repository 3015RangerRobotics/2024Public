package frc.robot.subsystems.arm_joint;

import edu.wpi.first.math.geometry.Rotation2d;

public class ArmJointIODummy implements ArmJointIO {
  public ArmJointIODummy() {}

  /**
   * Update the inputs for the arm joint
   *
   * @param inputs The inputs to update
   */
  @Override
  public void updateInputs(ArmJointInputs inputs) {}

  /**
   * Set the target angle for the arm joint
   *
   * @param angle Target arm angle
   */
  @Override
  public void setTargetAngle(Rotation2d angle) {}

  @Override
  public void setTargetAngle(Rotation2d angle, double maxVel, double maxAccel) {}

  /**
   * Set the voltage output to the arm joint motors
   *
   * @param volts Voltage to output
   */
  @Override
  public void setVoltage(double volts) {}

  @Override
  public void brakeMotors() {}
}
