package frc.lib.input.controllers.rumble;

public class RumbleOn extends RumbleAnimation {
  private final double intensity;

  public RumbleOn(double intensity) {
    this.intensity = intensity;
  }

  public RumbleOn() {
    this(1.0);
  }

  @Override
  public double getRumbleOutput(double timeSeconds) {
    return intensity;
  }
}
