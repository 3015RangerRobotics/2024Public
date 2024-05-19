package frc.lib.input.controllers.rumble;

public class RumbleOff extends RumbleAnimation {
  @Override
  public double getRumbleOutput(double timeSeconds) {
    return 0.0;
  }
}
