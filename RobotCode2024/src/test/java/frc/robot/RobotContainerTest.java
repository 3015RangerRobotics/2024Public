package frc.robot;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class RobotContainerTest {
  @BeforeEach
  public void setup() {
    HAL.initialize(500, 0);
  }

  @Test
  public void doesNotBlowUp() {
    RobotContainer.init();
  }
}
