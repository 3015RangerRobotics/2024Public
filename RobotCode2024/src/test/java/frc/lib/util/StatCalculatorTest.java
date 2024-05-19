package frc.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class StatCalculatorTest {
  private static final double DELTA = 1E-2;

  @Test
  public void testSize() {
    StatCalculator calc = new StatCalculator();

    calc.addNumber(1);
    calc.addNumber(2);
    calc.addNumber(3);

    assertEquals(3, calc.getSize());

    calc.clear();

    assertEquals(0, calc.getSize());
  }

  @Test
  public void testSum() {
    StatCalculator calc = new StatCalculator();

    assertEquals(0, calc.getSum(), DELTA);

    calc.addNumber(1);
    calc.addNumber(2);
    calc.addNumber(3);

    assertEquals(6, calc.getSum(), DELTA);

    calc.addNumber(2);

    assertEquals(8, calc.getSum(), DELTA);
  }

  @Test
  public void testMean() {
    StatCalculator calc = new StatCalculator();

    assertTrue(Double.isNaN(calc.getMean()));

    calc.addNumber(5);
    calc.addNumber(10);
    calc.addNumber(15);

    assertEquals(10, calc.getMean(), DELTA);

    calc.addNumber(2);

    assertEquals(8, calc.getMean(), DELTA);
  }

  @Test
  public void testStdDev() {
    StatCalculator calc = new StatCalculator();

    calc.addNumber(5);

    assertTrue(Double.isNaN(calc.getStandardDeviation()));

    calc.addNumber(10);
    calc.addNumber(11);
    calc.addNumber(12);
    calc.addNumber(13);
    calc.addNumber(14);
    calc.addNumber(15);
    calc.addNumber(20);

    assertEquals(4.3, calc.getStandardDeviation(), DELTA);

    calc.addNumber(0);

    assertEquals(5.8, calc.getStandardDeviation(), DELTA);
  }

  @Test
  public void testMedian() {
    StatCalculator calc = new StatCalculator();

    assertTrue(Double.isNaN(calc.getMedian()));

    calc.addNumber(7);
    calc.addNumber(4);
    calc.addNumber(9);

    assertEquals(7, calc.getMedian(), DELTA);

    calc.addNumber(13);

    assertEquals(8, calc.getMedian(), DELTA);

    calc.addNumber(7.45);

    assertEquals(7.45, calc.getMedian(), DELTA);
  }

  @Test
  public void testMinMax() {
    StatCalculator calc = new StatCalculator();

    calc.addNumber(3);
    calc.addNumber(7);

    assertEquals(3, calc.getLowestValue());
    assertEquals(7, calc.getHighestValue());
  }
}
