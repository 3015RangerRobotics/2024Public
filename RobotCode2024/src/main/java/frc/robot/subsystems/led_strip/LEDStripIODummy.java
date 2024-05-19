package frc.robot.subsystems.led_strip;

import frc.lib.subsystem.AdvancedSubsystem;

public class LEDStripIODummy implements LEDStripIO {
  @Override
  public void setStatusLED(AdvancedSubsystem.SystemStatus status, int led) {}

  @Override
  public void ledStripOff() {}

  @Override
  public void partyAnimation() {}

  @Override
  public void sadAnimation() {}

  @Override
  public void breathingWhite() {}

  @Override
  public void redAllianceAnimation() {}

  @Override
  public void blueAllianceAnimation() {}

  @Override
  public void fireAnimation() {}
}
