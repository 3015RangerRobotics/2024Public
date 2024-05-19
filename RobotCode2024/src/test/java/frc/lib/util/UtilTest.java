package frc.lib.util;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

public class UtilTest {
  private static final double DELTA = 1E-5;

  @Test
  public void multiplyTwist3d() {
    Twist3d toMult = new Twist3d(0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
    Twist3d mult = Util.multiplyTwist(toMult, 3.0);

    assertEquals(1.5, mult.dx, DELTA);
    assertEquals(1.5, mult.dy, DELTA);
    assertEquals(1.5, mult.dz, DELTA);
    assertEquals(1.5, mult.rx, DELTA);
    assertEquals(1.5, mult.ry, DELTA);
    assertEquals(1.5, mult.rz, DELTA);
  }

  @Test
  public void epsilonEqualsDouble() {
    assertTrue(Util.epsilonEquals(0.5, 0.5, 0.1));
    assertTrue(Util.epsilonEquals(0.5, 0.6, 0.1));

    assertTrue(Util.epsilonEquals(0.5, 0.5, 0.01));
    assertFalse(Util.epsilonEquals(0.5, 0.6, 0.01));

    assertTrue(Util.epsilonEquals(0.0, 1E-10));
    assertFalse(Util.epsilonEquals(0.0, 1E-5));
  }

  @Test
  public void epsilonEqualsChassisSpeeds() {
    assertTrue(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(1E-10, 0, 0)));
    assertTrue(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(0, 1E-10, 0)));
    assertTrue(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(0, 0, 1E-10)));

    assertFalse(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(1E-5, 0, 0)));
    assertFalse(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(0, 1E-5, 0)));
    assertFalse(Util.epsilonEquals(new ChassisSpeeds(), new ChassisSpeeds(0, 0, 1E-5)));
  }
}
