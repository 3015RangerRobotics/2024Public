package frc.robot.subsystems.led_strip;

import com.ctre.phoenix.led.*;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.robot.Constants;

public class LEDStripIOCANdle implements LEDStripIO {
  private final CANdle candle;

  private final RainbowAnimation rainbowAnimation0 =
      new RainbowAnimation(1.0, 1.0, Constants.LEDStrip.stripLED, false, 8);
  private final RainbowAnimation rainbowAnimation1 =
      new RainbowAnimation(
          1.0, 1.0, Constants.LEDStrip.stripLED, false, 8 + Constants.LEDStrip.stripLED);

  private final StrobeAnimation sadAnimation0 =
      new StrobeAnimation(255, 0, 0, 0, 1.0, Constants.LEDStrip.stripLED, 8);
  private final StrobeAnimation sadAnimation1 =
      new StrobeAnimation(
          255, 0, 0, 0, 1.0, Constants.LEDStrip.stripLED, 8 + Constants.LEDStrip.stripLED);

  private final SingleFadeAnimation whiteAnimation0 =
      new SingleFadeAnimation(255, 255, 255, 0, 0.5, Constants.LEDStrip.stripLED, 8);
  private final SingleFadeAnimation whiteAnimation1 =
      new SingleFadeAnimation(
          255, 255, 255, 0, 0.5, Constants.LEDStrip.stripLED, 8 + Constants.LEDStrip.stripLED);

  private final LarsonAnimation redAnimation0 =
      new LarsonAnimation(
          255, 0, 0, 0, 0.03, Constants.LEDStrip.stripLED, LarsonAnimation.BounceMode.Center, 2, 8);
  private final LarsonAnimation redAnimation1 =
      new LarsonAnimation(
          255,
          0,
          0,
          0,
          0.03,
          Constants.LEDStrip.stripLED,
          LarsonAnimation.BounceMode.Center,
          2,
          8 + Constants.LEDStrip.stripLED);

  private final LarsonAnimation blueAnimation0 =
      new LarsonAnimation(
          0, 0, 255, 0, 0.03, Constants.LEDStrip.stripLED, LarsonAnimation.BounceMode.Center, 2, 8);
  private final LarsonAnimation blueAnimation1 =
      new LarsonAnimation(
          0,
          0,
          255,
          0,
          0.03,
          Constants.LEDStrip.stripLED,
          LarsonAnimation.BounceMode.Center,
          2,
          8 + Constants.LEDStrip.stripLED);

  private final FireAnimation fireAnimation0 =
      new FireAnimation(0.8, 0.75, Constants.LEDStrip.stripLED, 1.0, 0.3, false, 8);
  private final FireAnimation fireAnimation1 =
      new FireAnimation(
          0.8, 0.75, Constants.LEDStrip.stripLED, 1.0, 0.3, false, 8 + Constants.LEDStrip.stripLED);

  public LEDStripIOCANdle() {
    candle = new CANdle(Constants.LEDStrip.candleID, Constants.canivoreBusName);

    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = CANdle.LEDStripType.GRB;
    config.v5Enabled = true;
    config.disableWhenLOS = true;
    config.statusLedOffWhenActive = true;
    config.vBatOutputMode = CANdle.VBatOutputMode.Off;

    candle.configAllSettings(config);
  }

  @Override
  public void setStatusLED(AdvancedSubsystem.SystemStatus status, int led) {
    switch (status) {
      case OK:
        // Led Green
        candle.setLEDs(0, 50, 0, 0, led, 1);
        break;
      case WARNING:
        // Led Yellow
        candle.setLEDs(50, 35, 0, 0, led, 1);
        break;
      case ERROR:
        // Led Red
        candle.setLEDs(50, 0, 0, 0, led, 1);
        break;
    }
  }

  @Override
  public void ledStripOff() {
    candle.clearAnimation(0);
    candle.clearAnimation(1);
  }

  @Override
  public void partyAnimation() {
    candle.animate(rainbowAnimation0, 0);
    candle.animate(rainbowAnimation1, 1);
  }

  @Override
  public void sadAnimation() {
    candle.animate(sadAnimation0, 0);
    candle.animate(sadAnimation1, 1);
  }

  @Override
  public void breathingWhite() {
    candle.animate(whiteAnimation0, 0);
    candle.animate(whiteAnimation1, 1);
  }

  @Override
  public void redAllianceAnimation() {
    candle.animate(redAnimation0, 0);
    candle.animate(redAnimation1, 1);
  }

  @Override
  public void blueAllianceAnimation() {
    candle.animate(blueAnimation0, 0);
    candle.animate(blueAnimation1, 1);
  }

  @Override
  public void fireAnimation() {
    candle.animate(fireAnimation0, 0);
    candle.animate(fireAnimation1, 1);
  }
}
