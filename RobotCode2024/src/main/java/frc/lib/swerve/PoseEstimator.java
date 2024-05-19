package frc.lib.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.NoSuchElementException;
import java.util.Optional;

public class PoseEstimator {
  private static final double poseBufferSizeSeconds = 1.0;

  private final TimeInterpolatableBuffer<Pose2d> poseBuffer;
  private final Matrix<N3, N1> odometryStdDevs;
  private final SwerveDriveKinematics kinematics;

  private SwerveDriveWheelPositions lastWheelPositions =
      new SwerveDriveWheelPositions(
          new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
          });
  private Rotation2d lastGyroRotation = new Rotation2d();
  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();

  public PoseEstimator(Matrix<N3, N1> odometryStdDevs, SwerveDriveKinematics kinematics) {
    this.poseBuffer = TimeInterpolatableBuffer.createBuffer(poseBufferSizeSeconds);
    this.odometryStdDevs = odometryStdDevs;
    this.kinematics = kinematics;
  }

  public void addOdometryObservation(OdometryObservation observation) {
    Pose2d lastOdomPose = odometryPose;
    Twist2d kinematicsTwist =
        kinematics.toTwist2d(lastWheelPositions, observation.wheelPositions());
    lastWheelPositions = observation.wheelPositions();

    Rotation2d rotationDiff = observation.gyroRotation().minus(lastGyroRotation);
    Twist2d twist = new Twist2d(kinematicsTwist.dx, kinematicsTwist.dy, rotationDiff.getRadians());
    lastGyroRotation = observation.gyroRotation();

    odometryPose = odometryPose.exp(twist);
    poseBuffer.addSample(observation.timestamp(), odometryPose);
    estimatedPose = estimatedPose.exp(lastOdomPose.log(odometryPose));
  }

  public void addVisionObservation(VisionObservation observation) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - poseBufferSizeSeconds
          > observation.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }

    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) {
      return;
    }

    Transform2d sampleToOdomTransform = new Transform2d(sample.get(), odometryPose);
    Transform2d odomToSampleTransform = new Transform2d(odometryPose, sample.get());

    Pose2d estimateAtTime = estimatedPose.plus(odomToSampleTransform);

    var r = new double[3];
    for (int i = 0; i < 3; i++) {
      r[i] = observation.stdDevs().get(i, 0) * observation.stdDevs().get(i, 0);
    }

    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; row++) {
      double stdDev = odometryStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }

    Twist2d twist = estimateAtTime.log(observation.visionPose());

    var twistMatrix = visionK.times(VecBuilder.fill(twist.dx, twist.dy, twist.dtheta));

    Twist2d scaledTwist =
        new Twist2d(twistMatrix.get(0, 0), twistMatrix.get(1, 0), twistMatrix.get(2, 0));

    estimatedPose = estimateAtTime.exp(scaledTwist).plus(sampleToOdomTransform);
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    odometryPose = pose;
    poseBuffer.clear();
  }

  public Pose2d getOdomPose() {
    return odometryPose;
  }

  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public Optional<Pose2d> sampleAt(double timestamp) {
    return poseBuffer
        .getSample(timestamp)
        .map(sample -> sample.plus(new Transform2d(odometryPose, estimatedPose)));
  }

  public record OdometryObservation(
      SwerveDriveWheelPositions wheelPositions, Rotation2d gyroRotation, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
