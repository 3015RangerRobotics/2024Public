package frc.lib.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.Util;
import org.junit.jupiter.api.Test;

public class SwerveSetpointGeneratorTest {

  protected static final double kRobotSide = 0.616; // m
  protected static final SwerveDriveKinematics kKinematics =
      new SwerveDriveKinematics(
          // Front left
          new Translation2d(kRobotSide / 2.0, kRobotSide / 2.0),
          // Front right
          new Translation2d(kRobotSide / 2.0, -kRobotSide / 2.0),
          // Back left
          new Translation2d(-kRobotSide / 2.0, kRobotSide / 2.0),
          // Back right
          new Translation2d(-kRobotSide / 2.0, -kRobotSide / 2.0));
  protected static final SwerveKinematicLimits kKinematicLimits =
      new SwerveKinematicLimits(5.0, 10.0, Math.toRadians(1500.0));
  protected static final double kDt = 0.01; // s
  protected static final double kMaxSteeringVelocityError = Math.toRadians(2.0); // rad/s
  protected static final double kMaxAccelerationError = 0.01; // m/s^2

  public static void satisfiesConstraints(
      SwerveSetpoint prev, SwerveSetpoint next, boolean checkAcceleration) {
    for (int i = 0; i < prev.moduleStates.length; ++i) {
      final var prevModule = prev.moduleStates[i];
      final var nextModule = next.moduleStates[i];
      Rotation2d diffRotation = prevModule.angle.unaryMinus().rotateBy(nextModule.angle);
      assertTrue(
          Math.abs(diffRotation.getRadians())
              < kKinematicLimits.maxSteerVelocity() + kMaxSteeringVelocityError);
      assertTrue(Math.abs(nextModule.speedMetersPerSecond) <= kKinematicLimits.maxDriveVelocity());
      assertTrue(
          Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt
              <= kKinematicLimits.maxDriveAcceleration() + kMaxAccelerationError);

      if (checkAcceleration) {
        // If we should check acceleration, check that we are reaching max acceleration at all
        // times.
        assertEquals(
            Math.abs(nextModule.speedMetersPerSecond - prevModule.speedMetersPerSecond) / kDt,
            kKinematicLimits.maxDriveAcceleration(),
            kMaxAccelerationError);
      }
    }
  }

  //  public static SwerveSetpoint driveToGoal(
  //      SwerveSetpoint prevSetpoint, ChassisSpeeds goal, SwerveSetpointGenerator generator) {
  //    return driveToGoal(prevSetpoint, goal, generator, false);
  //  }

  //  public static SwerveSetpoint driveToGoal(
  //      SwerveSetpoint prevSetpoint,
  //      ChassisSpeeds goal,
  //      SwerveSetpointGenerator generator,
  //      boolean checkAcceleration) {
  //    System.out.println("Driving to goal state " + goal);
  //    System.out.println("Initial state: " + prevSetpoint);
  //    while (!Util.epsilonEquals(prevSetpoint.chassisSpeeds, new ChassisSpeeds())) {
  //      var newsetpoint = generator.generateSetpoint(kKinematicLimits, prevSetpoint, goal, kDt);
  //      System.out.println(newsetpoint);
  //      satisfiesConstraints(prevSetpoint, newsetpoint, checkAcceleration);
  //      prevSetpoint = newsetpoint;
  //    }
  //    return prevSetpoint;
  //  }

  @Test
  public void testAccelerationLimits() {
    SwerveModuleState[] initialStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

    var generator = new SwerveSetpointGenerator(kKinematics);

    // Just drive straight
    var goalSpeeds = new ChassisSpeeds(5.0, 0.0, 0.0);

    var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
    satisfiesConstraints(setpoint, newsetpoint, true);
  }

  @Test
  public void testGenerateSetpoint() {
    SwerveModuleState[] initialStates = {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
    };
    SwerveSetpoint setpoint = new SwerveSetpoint(new ChassisSpeeds(), initialStates);

    var generator = new SwerveSetpointGenerator(kKinematics);

    var goalSpeeds = new ChassisSpeeds(0.0, 0.0, 1.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, -1.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(0.0, 1.0, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(0.1, -1.0, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(1.0, -0.5, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }

    goalSpeeds = new ChassisSpeeds(1.0, 0.4, 0.0);
    while (!Util.epsilonEquals(setpoint.chassisSpeeds, goalSpeeds)) {
      var newsetpoint = generator.generateSetpoint(kKinematicLimits, setpoint, goalSpeeds, kDt);
      satisfiesConstraints(setpoint, newsetpoint, false);
      setpoint = newsetpoint;
    }
  }
}
