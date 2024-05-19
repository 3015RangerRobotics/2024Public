package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Nat;
import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

public class Vector3Test {
  private static final double DELTA = 1E-2;

  @Test
  public void testVector3() {
    Vector3 vec1 = new Vector3(1, 2, 3);

    assertEquals(1, vec1.getX(), DELTA);
    assertEquals(2, vec1.getY(), DELTA);
    assertEquals(3, vec1.getZ(), DELTA);

    Vector3 vec2 = new Vector3(new SimpleMatrix(3, 1, true, new double[] {1, 2, 3}));

    assertEquals(1, vec2.getX(), DELTA);
    assertEquals(2, vec2.getY(), DELTA);
    assertEquals(3, vec2.getZ(), DELTA);

    Vector3 vec3 = new Vector3(Nat.N3());

    assertEquals(0, vec3.getX(), DELTA);
    assertEquals(0, vec3.getY(), DELTA);
    assertEquals(0, vec3.getZ(), DELTA);
  }
}
