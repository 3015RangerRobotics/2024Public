package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.swerve.SwerveKinematicLimits;
import frc.lib.swerve.SwerveSetpointGenerator;
import frc.robot.subsystems.swerve_module.ModuleConfig;
import java.io.IOException;
import org.photonvision.simulation.SimCameraProperties;

public final class Constants {
  public static final boolean competitionMode = false;
  public static final boolean logReplay = false;
  public static final String canivoreBusName = "*";
  public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);
  public static final AprilTagFieldLayout apriltagLayout;

  static {
    try {
      apriltagLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  public static final Translation3d passPosBlue = new Translation3d(0.54, 7.25, 0.0);
  public static final Translation3d passPosRed = new Translation3d(16.0, 7.25, 0.0);

  public static final Translation3d speakerPosBlue = new Translation3d(0.19, 5.55, 2.1);
  public static final Translation3d speakerPosRed = new Translation3d(16.35, 5.55, 2.1);
  public static final double speakerZFarDiff = 0.0;

  public static final Pose2d targetAmpPoseBlue = new Pose2d(2.0, 7.62, Rotation2d.fromDegrees(90));
  public static final Pose2d targetAmpPoseRed =
      GeometryUtil.flipFieldPose(new Pose2d(1.8, 7.62, Rotation2d.fromDegrees(90)));
  public static final Pose2d targetAmpPathfindPoseBlue =
      new Pose2d(
          targetAmpPoseBlue.getTranslation().minus(new Translation2d(0.0, 0.5)),
          targetAmpPoseBlue.getRotation());
  public static final Pose2d targetAmpPathfindPoseRed =
      GeometryUtil.flipFieldPose(targetAmpPathfindPoseBlue);

  public static final Pose2d chutePoseBlueFar = new Pose2d(16.0, 1.0, Rotation2d.fromDegrees(-60));
  public static final Pose2d chutePoseRedFar = GeometryUtil.flipFieldPose(chutePoseBlueFar);
  public static final Pose2d chutePoseBlueMid = new Pose2d(15.45, 0.8, Rotation2d.fromDegrees(-60));
  public static final Pose2d chutePoseRedMid = GeometryUtil.flipFieldPose(chutePoseBlueMid);
  public static final Pose2d chutePoseBlueNear = new Pose2d(15.1, 0.7, Rotation2d.fromDegrees(-60));
  public static final Pose2d chutePoseRedNear = GeometryUtil.flipFieldPose(chutePoseBlueNear);
  public static final Pose2d targetChutePathfindPoseBlue =
      new Pose2d(15.15, 1.7, chutePoseBlueFar.getRotation());

  //  public static final Pose2d targetChutePathfindPoseBlue =
  //      new Pose2d(15.15, 1.7, Rotation2d.fromDegrees(150));
  public static final Pose2d targetChutePathfindPoseRed =
      GeometryUtil.flipFieldPose(targetChutePathfindPoseBlue);

  public static final Pose2d speakerShootPoseBlue =
      new Pose2d(3.5, 6.0, Rotation2d.fromDegrees(-170.0));
  public static final Pose2d speakerShootPoseRed = GeometryUtil.flipFieldPose(speakerShootPoseBlue);

  public static final Pose2d climbLeftPathfindingPoseBlue =
      new Pose2d(3.9, 5.63, Rotation2d.fromDegrees(120));
  public static final Pose2d climbCenterPathfindingPoseBlue =
      new Pose2d(6.5, 4.05, Rotation2d.fromDegrees(0));
  public static final Pose2d climbRightPathfindingPoseBlue =
      new Pose2d(3.9, 2.57, Rotation2d.fromDegrees(-120));
  public static final Pose2d climbLeftPathfindingPoseRed =
      GeometryUtil.flipFieldPose(climbRightPathfindingPoseBlue);
  public static final Pose2d climbCenterPathfindingPoseRed =
      GeometryUtil.flipFieldPose(climbCenterPathfindingPoseBlue);
  public static final Pose2d climbRightPathfindingPoseRed =
      GeometryUtil.flipFieldPose(climbLeftPathfindingPoseBlue);

  public static final Pose2d climbLeftPoseBlue = new Pose2d(4.5, 4.75, Rotation2d.fromDegrees(120));
  public static final Pose2d climbCenterPoseBlue =
      new Pose2d(5.65, 4.05, Rotation2d.fromDegrees(0));
  public static final Pose2d climbRightPoseBlue =
      new Pose2d(4.5, 3.45, Rotation2d.fromDegrees(-120));
  public static final Pose2d climbLeftPoseRed = GeometryUtil.flipFieldPose(climbRightPoseBlue);
  public static final Pose2d climbCenterPoseRed = GeometryUtil.flipFieldPose(climbCenterPoseBlue);
  public static final Pose2d climbRightPoseRed = GeometryUtil.flipFieldPose(climbLeftPoseBlue);

  public static final PathConstraints pathfindingConstraints =
      new PathConstraints(4.0, 3.0, Units.degreesToRadians(270.0), Units.degreesToRadians(270.0));

  public static final class Localization {
    public static final double fieldBorderMargin = 0.25;
    public static final double zMargin = 0.5;
    public static final double xyStdDevCoefficient = 0.02;
    public static final double thetaStdDevCoefficient = 0.04;
    public static final double ambiguityThreshold = 0.15;

    public static final Transform3d robotToPolaris1FL =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(12.979),
                Units.inchesToMeters(10.06),
                Units.inchesToMeters(11.20134)),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(20)));
    public static final Transform3d robotToPolaris2FR =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(10.716),
                -Units.inchesToMeters(12.323),
                Units.inchesToMeters(11.20134)),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-20)));
    public static final Transform3d robotToPolaris3BL =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(10.716),
                Units.inchesToMeters(12.323),
                Units.inchesToMeters(11.20134)),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(160)));
    public static final Transform3d robotToPolaris4BR =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(10.716),
                -Units.inchesToMeters(12.323),
                Units.inchesToMeters(11.20134)),
            new Rotation3d(0.0, Units.degreesToRadians(-30.0), Units.degreesToRadians(-160)));

    public static final class SimProperties {
      public static final SimCameraProperties polarisProperties = new SimCameraProperties();

      static {
        polarisProperties.setCalibration(1600, 1200, Rotation2d.fromDegrees(93.75));
        polarisProperties.setCalibError(0.5, 0.3);
        polarisProperties.setFPS(50);
        polarisProperties.setAvgLatencyMs(15);
        polarisProperties.setLatencyStdDevMs(2);
      }
    }
  }

  public static final class LimelightNotes {
    public static final Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(
                -0.1928876 - Units.inchesToMeters(5.5), 0.0, 0.2374392 + Units.inchesToMeters(3.0)),
            new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(180)));
  }

  public static final class Swerve {
    public static final int imuCanID = 1;
    public static final double odomUpdateHz = 250.0;

    public static final double maxVelTele = 5.5;
    public static final double maxAngularVelTele = Units.degreesToRadians(270);
    public static final double teleAngleHoldFactor = 4.0;
    public static final double teleStickRateLimit = 4.0;

    public static final ModuleConfig frontLeftModuleConfig =
        new ModuleConfig(
            7,
            8,
            4,
            new Translation2d(Units.inchesToMeters(10.75), Units.inchesToMeters(10.75)),
            "FLModule");

    public static final ModuleConfig frontRightModuleConfig =
        new ModuleConfig(
            1,
            2,
            1,
            new Translation2d(Units.inchesToMeters(10.75), Units.inchesToMeters(-10.75)),
            "FRModule");

    public static final ModuleConfig backLeftModuleConfig =
        new ModuleConfig(
            5,
            6,
            3,
            new Translation2d(Units.inchesToMeters(-10.75), Units.inchesToMeters(10.75)),
            "BLModule");

    public static final ModuleConfig backRightModuleConfig =
        new ModuleConfig(
            3,
            4,
            2,
            new Translation2d(Units.inchesToMeters(-10.75), Units.inchesToMeters(-10.75)),
            "BRModule");

    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            frontLeftModuleConfig.moduleOffset(),
            frontRightModuleConfig.moduleOffset(),
            backLeftModuleConfig.moduleOffset(),
            backRightModuleConfig.moduleOffset());
    public static final SwerveSetpointGenerator setpointGenerator =
        new SwerveSetpointGenerator(kinematics);

    public static final HolonomicPathFollowerConfig pathFollowingConfig =
        new HolonomicPathFollowerConfig(
            new PIDConstants(6.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0),
            SwerveModule.driveMaxVel,
            frontLeftModuleConfig.moduleOffset().getNorm(),
            new ReplanningConfig(true, true, 0.5, 0.25));

    public static final PIDConstants aimAtGoalConstants =
        new PIDConstants(6.0, 0.0, 0.0, Units.degreesToRadians(20));

    public static final class PoseEstimator {
      public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.003, 0.003, 0.0002);
    }
  }

  public static final class SwerveModule {
    public static final double driveGearing = 5.1429;
    public static final double rotationGearing = 12.8;

    public static final double wheelDiameter = 3.75;
    public static final double driveRotationsPerMeter =
        driveGearing / (Math.PI * Units.inchesToMeters(wheelDiameter));

    public static final double driveMaxVel = 5.5;

    public static final double drivekP = 0.3;
    public static final double drivekI = 0.0;
    public static final double drivekD = 0.0;
    public static final double drivekV = 0.14;
    public static final double drivekA = 0.012754;
    public static final double drivekS = 0.0; // 0.1827;

    public static final double rotationkP = 100.0;
    public static final double rotationkI = 0.0;
    public static final double rotationkD = 0.2;
    public static final double rotationkV = 1.5835;
    public static final double rotationkA = 0.51867;
    public static final double rotationkS = 0.0029337;

    public static final double driveCurrentLimit = 100.0;
    public static final double rotationCurrentLimit = 60.0;

    public static final double rotationMMAccel = 100.0;
    public static final double rotationMMCruiseVel = 10.0;

    public static final SwerveKinematicLimits kinematicLimits =
        new SwerveKinematicLimits(driveMaxVel, 10.0, Units.rotationsToRadians(rotationMMCruiseVel));
  }

  public static final class Lidar {
    public static final double maxDistance = 4.0;

    public static final double clusterDistanceThreshold = 0.5;
    public static final double clusterMinCount = 5;
    public static final double fieldWallThreshold = 0.5;

    public static final Transform3d robotToLidarOrange =
        new Transform3d(
            new Translation3d(-0.3, -0.3, 0.3),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(-45.0)));
    public static final Transform3d robotToLidarRaspberry =
        new Transform3d(
            new Translation3d(-0.31, 0.32, 0.3),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(135.0)));
  }

  public static final class ShooterJoint {
    public static final int motorID = 10;
    public static final int encoderID = 10;

    public static final double kP = 100.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 5.6639;
    public static final double kA = 0.1641;
    public static final double kS = 0.8151;

    public static final double gearing = 70.0;

    public static final double mmMaxVel = 1.5;
    public static final double mmMaxAccel = 5.0;

    public static final Rotation2d forwardLimit = Rotation2d.fromDegrees(74);
    public static final Rotation2d reverseLimit = Rotation2d.fromRotations(-70);

    public static final double currentLimit = 40.0;

    public static final Rotation2d ampPositionAngle = Rotation2d.fromDegrees(20);
    public static final Rotation2d ampPositionAngleBlue = Rotation2d.fromDegrees(20);
    public static final Rotation2d passPositionAngle = Rotation2d.fromDegrees(20);
    public static final Rotation2d trapPositionAngle = Rotation2d.fromDegrees(74);
    public static final Rotation2d chutePositionAngle = Rotation2d.fromDegrees(-16);
    public static final Rotation2d preclimbPositionAngle = Rotation2d.fromDegrees(60);
    public static final Rotation2d postclimbPositionAngle = Rotation2d.fromDegrees(50);
    public static final Rotation2d intakePositionAngle = Rotation2d.fromDegrees(-12);
    public static final Rotation2d fenderPositionAngle = Rotation2d.fromDegrees(25);

    public static final Rotation2d farAutoAngle = Rotation2d.fromDegrees(52.85);
    public static final Rotation2d sourceAutoAngle = Rotation2d.fromDegrees(52.1);

    public static final class SimInfo {
      public static final double armLength = Units.inchesToMeters(16.0);
      public static final double armMass = 3.0; // kg
      public static final double MOI = SingleJointedArmSim.estimateMOI(armLength, armMass);
    }
  }

  public static final class Intake {
    public static final int motorID = 11;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12151;
    public static final double kA = 0.0042446;
    public static final double kS = 0.24808;

    public static final double gearing = 3.0;

    public static final double currentLimit = 60.0;

    public static final double rollerCircumference = Math.PI * Units.inchesToMeters(2.0);
  }

  public static final class Shooter {
    public static final int topMotorID = 12;
    public static final int bottomMotorID = 13;

    public static final double gearing = 1.0;

    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.1137;
    public static final double kA = 0.025383;
    public static final double kS = 0.028757;
    public static final double frictionTorque = 12.0;

    public static final double shooterWheelSpeed = 25.0;
    public static final double shooterExitSpeed = 15.0;
    public static final double surfaceSpeedMultiplier = shooterWheelSpeed / shooterExitSpeed;
    public static final Rotation2d exitAngleOffset =
        Rotation2d.fromDegrees(0.0); // Bigger = more up, 4.25
    public static final double passSurfaceSpeedMultiplier = 1.45;
    public static final double passMinSpeed = 6.0;
    public static final double passMaxSpeed = 11.0;

    public static final double rollerCircumference = Math.PI * Units.inchesToMeters(4.0);

    public static final double currentLimit = 200.0;

    public static final Transform3d shooterToNote =
        new Transform3d(
            new Translation3d(-0.255, 0.0, 0.25),
            new Rotation3d(0.0, Units.degreesToRadians(-90), 0.0));

    public static final double slowShootRPM = 1000.0;
    public static final double passRPM = 2700.0;

    public static final double chuteIntakeRPM = -1200.0;
  }

  public static final class ArmJoint {
    public static final int motorID = 14;
    public static final int followerMotorID = 15;

    public static final int encoderID = 11;

    public static final double kP = 200.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 10.657;
    public static final double kA = 1.2939;
    public static final double kS = 0.81888;
    public static final double kG = 0.047416;

    public static final double gearing = 3125.0 / 27.0;

    public static final double mmMaxVel = 1.1;
    public static final double mmMaxAccel = 2.5; // 7.0;
    public static final double mmMaxJerk = 100.0;

    public static final Rotation2d forwardLimit = Rotation2d.fromDegrees(180);
    public static final Rotation2d reverseLimit = Rotation2d.fromDegrees(13);

    public static final double currentLimit = 60.0;

    public static final Rotation2d ampPositionAngle = Rotation2d.fromDegrees(100); // 100
    public static final Rotation2d trapPositionAngle = Rotation2d.fromDegrees(80); // 80 at pitt
    public static final Rotation2d chutePositionAngle = Rotation2d.fromDegrees(86.5);
    public static final Rotation2d preclimbPositionAngle = Rotation2d.fromDegrees(110);
    public static final Rotation2d restPositionAngle = Rotation2d.fromDegrees(12);
    public static final Rotation2d highShotAngle = Rotation2d.fromDegrees(60);

    public static final class SimInfo {
      public static final double armLength = Units.inchesToMeters(20.0);
      public static final double armMass = 12.0; // kg
      public static final double MOI = SingleJointedArmSim.estimateMOI(armLength, armMass);
    }
  }

  public static final class Climber {
    public static final int leftMotorID = 16;
    public static final int rightMotorID = 17;

    public static final double metersPerRotation = Units.inchesToMeters(0.5);

    public static final double kP = 5.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.12021;
    public static final double kA = 0.0012079;
    public static final double kS = 0.25725;

    public static final double mmMaxVel = 1.2 / metersPerRotation;
    public static final double mmMaxAccel = 50.0 / metersPerRotation;

    public static final double forwardLimit = Units.inchesToMeters(23.25);
    public static final double reverseLimit = 0.0;

    public static final double currentLimit = 200.0;

    public static final double climberPosExtend = Units.inchesToMeters(23.25);
    public static final double climberPosRetract = Units.inchesToMeters(1.0);
    public static final double climberPosSafety = Units.inchesToMeters(10.0);

    public static final double climberCurrentCheck = 50.0;
  }

  public static final class ArmExtension {
    public static final int motorID = 18;
    public static final double metersPerRotation = Units.inchesToMeters(0.5);

    public static final double kP = 1.5;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.11515;
    public static final double kA = 0.0017514;
    public static final double kS = 0.3794;

    public static final double mmMaxVel = 1.25 / metersPerRotation;
    public static final double mmMaxAccel = 50.0 / metersPerRotation;

    public static final double forwardLimit = 0.42;
    public static final double reverseLimit = 0.0;

    public static final double currentLimit = 60.0;

    public static final double ampPositionExtension = 0.3;
    public static final double trapPositionExtension = 0.38; // 0.38 at pitt
    public static final double chutePositionExtension = 0.15;
    public static final double preclimbPositionExtension = 0.35;
    public static final double intakePositionExtension = 0.04;
    public static final double shootPositionExtension = 0.0;
    public static final double highShotExtension = 0.35;
  }

  public static final class Uptake {
    public static int motorID = 19;

    public static final double ringSensorMaxDistance = 0.05;
    public static final double ringSensorMaxDistance2 = 0.1;

    public static final double gearing = 5.0;

    public static final double vel_kP = 0.1;
    public static final double vel_kI = 0.0;
    public static final double vel_kD = 0.0;
    public static final double pos_kP = 1.0;
    public static final double pos_kI = 0.0;
    public static final double pos_kD = 0.0;
    public static final double kV = 0.11792;
    public static final double kA = 0.0014698;
    public static final double kS = 0.24579;

    public static final double rollerCircumference = Math.PI * Units.inchesToMeters(2.0);

    public static final double currentLimit = 40.0;

    public static final double shootRPM = 700.0;

    public static final double uptakeIntakeRPM = 700.0; // 125.0;

    public static final double chuteUptakeRPM = -100.0;
  }

  public static final class LEDStrip {
    public static final int numLEDs = 56;
    public static final int stripLED = 28;

    public static final int candleID = 1;

    public static final int swerveLED = 0;
    public static final int armJointLED = 1;
    public static final int armExtensionLED = 2;
    public static final int shooterJointLED = 3;
    public static final int shooterLED = 4;
    public static final int intakeLED = 5;
    public static final int uptakeLED = 6;
    public static final int climberLED = 7;
  }
}
