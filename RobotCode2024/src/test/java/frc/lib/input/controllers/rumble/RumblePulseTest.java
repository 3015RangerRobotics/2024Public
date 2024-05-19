package frc.lib.input.controllers.rumble;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RumblePulseTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testRumblePulse() {
    RumblePulse test = new RumblePulse(1);

    assertEquals(0.0, test.getRumbleOutput(0), DELTA);
    assertEquals(0.0, test.getRumbleOutput(0.25), DELTA);
    assertEquals(1.0, test.getRumbleOutput(0.5), DELTA);
    assertEquals(1.0, test.getRumbleOutput(0.75), DELTA);
    assertEquals(0.0, test.getRumbleOutput(1.0), DELTA);
  }
}
