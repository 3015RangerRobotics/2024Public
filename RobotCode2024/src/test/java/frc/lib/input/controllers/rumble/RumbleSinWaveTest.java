package frc.lib.input.controllers.rumble;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class RumbleSinWaveTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testRumbleSinWave() {
    RumbleSinWave test = new RumbleSinWave(1);

    assertEquals(0.5, test.getRumbleOutput(0), DELTA);
    assertEquals(0.8, test.getRumbleOutput(0.1), DELTA);
    assertEquals(1.0, test.getRumbleOutput(0.25), DELTA);
    assertEquals(0.8, test.getRumbleOutput(0.4), DELTA);
    assertEquals(0.5, test.getRumbleOutput(0.5), DELTA);
    assertEquals(0.025, test.getRumbleOutput(0.7), DELTA);
    assertEquals(0.0, test.getRumbleOutput(0.75), DELTA);
    assertEquals(0.1, test.getRumbleOutput(0.85), DELTA);
    assertEquals(0.5, test.getRumbleOutput(1.0), DELTA);
  }
}
