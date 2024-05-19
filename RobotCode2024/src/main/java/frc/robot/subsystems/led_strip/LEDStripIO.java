package frc.robot.subsystems.led_strip;

import frc.lib.subsystem.AdvancedSubsystem;

public interface LEDStripIO {
  void setStatusLED(AdvancedSubsystem.SystemStatus status, int led);

  void ledStripOff();

  void partyAnimation();

  void sadAnimation();

  void breathingWhite();

  void redAllianceAnimation();

  void blueAllianceAnimation();

  void fireAnimation();
}
