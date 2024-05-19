package frc.lib.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.ejml.simple.SimpleMatrix;

public class Vector3 extends Vector<N3> {
  public Vector3(Nat<N3> rows) {
    super(rows);
  }

  public Vector3(SimpleMatrix storage) {
    super(storage);
  }

  public Vector3(Matrix<N3, N1> other) {
    super(other);
  }

  public Vector3(double x, double y, double z) {
    this(VecBuilder.fill(x, y, z));
  }

  public double getX() {
    return this.get(0, 0);
  }

  public double getY() {
    return this.get(1, 0);
  }

  public double getZ() {
    return this.get(2, 0);
  }

  public Translation3d toTranslation() {
    return new Translation3d(getX(), getY(), getZ());
  }
}
