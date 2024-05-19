package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import org.junit.jupiter.api.Test;

public class InterpolatingTreeMapTest {
  @Test
  void testInterpolationDouble() {
    InterpolatingTreeMap<Double, Double> table =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Interpolator.forDouble());

    table.put(125.0, 450.0);
    table.put(200.0, 510.0);
    table.put(268.0, 525.0);
    table.put(312.0, 550.0);
    table.put(326.0, 650.0);

    // Key below minimum gives the smallest value
    assertEquals(450.0, table.get(100.0));

    // Minimum key gives exact value
    assertEquals(450.0, table.get(125.0));

    // Key gives interpolated value
    assertEquals(480.0, table.get(162.5));

    // Key at right of interpolation range gives exact value
    assertEquals(510.0, table.get(200.0));

    // Maximum key gives exact value
    assertEquals(650.0, table.get(326.0));

    // Key above maximum gives largest value
    assertEquals(650.0, table.get(400.0));

    table.trimEntriesBefore(250.0);

    assertEquals(525.0, table.get(100.0));

    table.clear();

    assertNull(table.get(0.0));
  }
}
