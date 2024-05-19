package frc.lib.input.controllers.rumble;

public class RumblePulse extends RumbleAnimation {
  private final double pulsePeriod;
  private final double intensity;

  public RumblePulse(double pulsesPerSecond, double intensity) {
    this.pulsePeriod = 1.0 / pulsesPerSecond;
    this.intensity = intensity;
  }

  public RumblePulse(double pulsesPerSecond) {
    this(pulsesPerSecond, 1.0);
  }

  @Override
  public double getRumbleOutput(double timeSeconds) {
    if (timeSeconds % pulsePeriod < pulsePeriod / 2) {
      return 0;
    } else {
      return intensity;
    }
  }
}
