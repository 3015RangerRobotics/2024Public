package frc.lib.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class StatCalculator {
  private final List<Double> numbers;

  public StatCalculator() {
    numbers = new ArrayList<>();
  }

  public void clear() {
    numbers.clear();
  }

  public void addNumber(double number) {
    numbers.add(number);
  }

  public int getSize() {
    return numbers.size();
  }

  public double getSum() {
    double sum = 0;

    for (double d : numbers) {
      sum += d;
    }

    return sum;
  }

  public double getMean() {
    if (getSize() == 0) {
      return Double.NaN;
    } else {
      return getSum() / getSize();
    }
  }

  public double getStandardDeviation() {
    if (getSize() < 2) {
      return Double.NaN;
    } else {
      double mean = getMean();
      return Math.sqrt(
          numbers.stream().mapToDouble(x -> Math.pow(x - mean, 2)).sum() / (getSize() - 1));
    }
  }

  public double getLowestValue() {
    return Collections.min(numbers);
  }

  public double getHighestValue() {
    return Collections.max(numbers);
  }

  public double getMedian() {
    if (getSize() == 0) {
      return Double.NaN;
    }

    Collections.sort(numbers);

    if (getSize() % 2 == 1) {
      return numbers.get(((getSize() + 1) / 2) - 1);
    } else {
      double lower = numbers.get((getSize() / 2) - 1);
      double upper = numbers.get(getSize() / 2);

      return (lower + upper) / 2.0;
    }
  }
}
