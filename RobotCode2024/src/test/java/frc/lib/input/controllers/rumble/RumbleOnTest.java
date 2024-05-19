package frc.lib.input.controllers.rumble;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RumbleOnTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testRumbleOn() {
    RumbleOn test = new RumbleOn();

    assertEquals(1.0, test.getRumbleOutput(0), DELTA);
    assertEquals(1.0, test.getRumbleOutput(0.5), DELTA);
    assertEquals(1.0, test.getRumbleOutput(0.75), DELTA);
  }
}
