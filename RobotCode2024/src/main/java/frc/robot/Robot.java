// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.FRCNetComm;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.behaviorTree.BehaviorTreeCommand;
import frc.lib.behaviorTree.BehaviorTrees;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.robot.util.LaunchCalculator;
import frc.robot.util.LoggedDashboardInt;
import frc.robot.util.NoteSimulator;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autoCommand;
  private static final List<PeriodicFunction> periodicFunctions = new ArrayList<>();

  private static final LoggedDashboardInt targetClimbPos =
      new LoggedDashboardInt("/Dashboard/TargetClimbPos", 1);
  //  private static final LoggedDashboardInt scoringMode =
  //      new LoggedDashboardInt("/Dashboard/ScoringMode", 1);
  //  private static final LoggedDashboardInt pickupMode =
  //      new LoggedDashboardInt("/Dashboard/PickupMode", 0);
  //  private static final LoggedDashboardInt chutePos =
  //      new LoggedDashboardInt("/Dashboard/ChutePos", 0);
  private static int scoringMode = 1;
  private static int pickupMode = 0;
  private static int chutePos = 0;

  public static Pose3d currentZEDPoseRobot = new Pose3d();

  @Override
  public void robotInit() {
    HAL.report(
        FRCNetComm.tResourceType.kResourceType_Language, FRCNetComm.tInstances.kLanguage_Kotlin);

    DriverStation.silenceJoystickConnectionWarning(true);
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty", BuildConstants.DIRTY == 1 ? "Uncommited Changes" : "All Changes Commited");

    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
      Logger.addDataReceiver(new NT4Publisher());
    } else {
      if (Constants.logReplay) {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(
            new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay"), 0.025));
      } else {
        Logger.addDataReceiver(new NT4Publisher());
      }
    }

    Logger.start();

    Constants.apriltagLayout.setOrigin(
        AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

    RobotContainer.init();

    // Warmup the PPLib library
    FollowPathCommand.warmupCommand().schedule();
    PathfindingCommand.warmupCommand().schedule();

    RobotController.setBrownoutVoltage(6.3);
  }

  @Override
  public void robotPeriodic() {
    double startTime = Logger.getRealTimestamp();

    NoteSimulator.update();

    Pose3d armProximalPose = getArmProximalPose();
    Pose3d armDistalPose = getArmDistalPose(armProximalPose);
    Pose3d shooterPose = getShooterPose(armDistalPose);

    Pose3d notePose = shooterPose.transformBy(Constants.Shooter.shooterToNote);
    Pose2d robotPose = RobotContainer.swerve.getPose();

    // Not actually zed anymore
    currentZEDPoseRobot = shooterPose.transformBy(Constants.LimelightNotes.robotToCamera);

    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            RobotContainer.swerve.getCurrentSpeeds(), robotPose.getRotation());

    if (getScoringMode() == ScoringMode.Pass) {
      Translation3d passPos = isRedAlliance() ? Constants.passPosRed : Constants.passPosBlue;
      double d = passPos.toTranslation2d().getDistance(robotPose.getTranslation());

      double passSpeed =
          GeometryUtil.doubleLerp(
              Constants.Shooter.passMinSpeed,
              Constants.Shooter.passMaxSpeed,
              MathUtil.clamp(d / 12.25, 0.0, 1.0));

      Pose3d notePoseField =
          new Pose3d(robotPose)
              .transformBy(new Transform3d(notePose.getTranslation(), notePose.getRotation()));

      LaunchCalculator.updateLaunch(
          passPos, notePoseField.getTranslation(), passSpeed, fieldSpeeds, false);
    } else {
      Translation3d targetPos =
          isRedAlliance() ? Constants.speakerPosRed : Constants.speakerPosBlue;

      // Subtract from the aim point Z height based on distance from goal (maximum at 8m)
      double dist = targetPos.toTranslation2d().getDistance(robotPose.getTranslation());
      double zDiff = Constants.speakerZFarDiff * MathUtil.clamp(dist / 8.0, 0.0, 1.0);

      Pose3d notePoseField;

      if (RobotContainer.localization.getSpeakerBasedPose().isEmpty()) {
        notePoseField =
            new Pose3d(robotPose)
                .transformBy(new Transform3d(notePose.getTranslation(), notePose.getRotation()));
      } else {
        notePoseField =
            new Pose3d(RobotContainer.localization.getSpeakerBasedPose().get())
                .transformBy(new Transform3d(notePose.getTranslation(), notePose.getRotation()));
      }

      LaunchCalculator.updateLaunch(
          targetPos.minus(new Translation3d(0, 0, zDiff)),
          notePoseField.getTranslation(),
          Constants.Shooter.shooterExitSpeed,
          fieldSpeeds,
          true);
    }

    CommandScheduler.getInstance().run();

    Logger.recordOutput("RobotComponentPoses", armProximalPose, armDistalPose, shooterPose);

    Logger.recordOutput("SimNotePose", NoteSimulator.getFieldPose(shooterPose));
    Logger.recordOutput(
        "SimNoteTraj", NoteSimulator.getNoteTrajectory().toArray(new Translation3d[0]));

    for (PeriodicFunction f : periodicFunctions) {
      f.runIfReady();
    }

    SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime());
    Logger.recordOutput("BatteryVoltage", RobotController.getBatteryVoltage());

    CANBus.CANBusStatus canBusStatus = CANBus.getStatus(Constants.canivoreBusName);
    Logger.recordOutput("CANUtil", canBusStatus.BusUtilization * 100.0);

    //    List<LidarDetection> robots = RobotContainer.lidar.getCurrentRobotDetections();
    //    List<Pair<Translation2d, Translation2d>> obs = new ArrayList<>();
    //    for (LidarDetection robotDet : robots) {
    //      Translation2d robot = robotDet.boundingBoxCenter().toTranslation2d();
    //      obs.add(Pair.of(robot.plus(new Translation2d(1, 1)), robot.minus(new Translation2d(1,
    // 1))));
    //    }
    //    Pathfinding.setDynamicObstacles(obs, RobotContainer.swerve.getPose2d().getTranslation());

    SmartDashboard.putBoolean("IsRedAlliance", isRedAlliance());

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("RobotPeriodicMS", runtimeMS);
  }

  @Override
  public void simulationPeriodic() {
    double batteryVoltage =
        BatterySim.calculateDefaultBatteryLoadedVoltage(
            RobotContainer.shooterJoint.getTotalCurrentDraw());
    RoboRioSim.setVInVoltage(batteryVoltage);
  }

  public static Pose3d getArmProximalPose() {
    Rotation2d armAngle = RobotContainer.armJoint.getAngle();
    return new Pose3d(0.2032, 0.0, 0.31, new Rotation3d(0.0, armAngle.getRadians(), 0.0))
        .transformBy(new Transform3d(new Translation3d(-0.2032, 0.0, -0.31), new Rotation3d()));
  }

  public static Pose3d getArmDistalPose(Pose3d armProximalPose) {
    double armExtensionLength = RobotContainer.armExtension.getExtensionMeters();
    return armProximalPose.transformBy(
        new Transform3d(new Translation3d(-armExtensionLength, 0.0, 0.0), new Rotation3d()));
  }

  public static Pose3d getShooterPose(Pose3d armDistalPose, boolean withOffset) {
    Rotation2d shooterAngle = RobotContainer.shooterJoint.getAngle();
    if (withOffset) {
      shooterAngle = shooterAngle.plus(Constants.Shooter.exitAngleOffset);
    }
    return armDistalPose
        .transformBy(
            new Transform3d(
                new Translation3d(-0.1928876, 0.0, 0.2374392),
                new Rotation3d(0.0, shooterAngle.getRadians(), 0.0)))
        .transformBy(
            new Transform3d(new Translation3d(0.1928876, 0.0, -0.2374392), new Rotation3d()));
  }

  public static Pose3d getShooterPose(Pose3d armDistalPose) {
    return getShooterPose(armDistalPose, false);
  }

  @Override
  public void disabledInit() {
    RobotContainer.driver.setRumbleAnimation(new RumbleOff());
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.autoPreviewField.setRobotPose(RobotContainer.swerve.getPose());
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autoCommand = RobotContainer.getAutonomousCommand();

    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();

      new BehaviorTreeCommand(BehaviorTrees.loadTree("AutoUpperMech")).schedule();
      RobotContainer.autoRotateForceOff = false;
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  public static void addPeriodic(Runnable callback, double period) {
    periodicFunctions.add(new PeriodicFunction(callback, period));
  }

  public static boolean isRedAlliance() {
    return DriverStation.getAlliance()
        .filter(value -> value == DriverStation.Alliance.Red)
        .isPresent();
  }

  public static TargetClimbPos getTargetClimbPos() {
    return switch (targetClimbPos.get()) {
      case 0 -> TargetClimbPos.Left;
      case 2 -> TargetClimbPos.Right;
      default -> TargetClimbPos.Center;
    };
  }

  public static ScoringMode getScoringMode() {
    return switch (scoringMode) {
      case 0 -> ScoringMode.AMP;
      case 2 -> ScoringMode.Pass;
      default -> ScoringMode.Speaker;
    };
  }

  public static void setScoringMode(ScoringMode mode) {
    switch (mode) {
      case AMP -> scoringMode = 0;
      case Pass -> scoringMode = 2;
      default -> scoringMode = 1;
    }
    Logger.recordOutput("ScoringMode", scoringMode);
  }

  public static PickupMode getPickupMode() {
    if (pickupMode == 1) {
      return PickupMode.Floor;
    } else {
      return PickupMode.Chute;
    }
  }

  public static ChutePos getChutePos() {
    return switch (chutePos) {
      case 1 -> ChutePos.Mid;
      case 2 -> ChutePos.Near;
      default -> ChutePos.Far;
    };
  }

  public static void setChutePos(ChutePos pos) {
    switch (pos) {
      case Mid -> chutePos = 1;
      case Near -> chutePos = 2;
      default -> chutePos = 0;
    }
    Logger.recordOutput("ChutePos", chutePos);
  }

  public enum TargetClimbPos {
    Left,
    Center,
    Right
  }

  public enum ScoringMode {
    AMP,
    Speaker,
    Pass
  }

  public enum PickupMode {
    Chute,
    Floor
  }

  public enum ChutePos {
    Far,
    Mid,
    Near
  }

  private static class PeriodicFunction {
    private final Runnable callback;
    private final double periodSeconds;

    private double lastRunTimeSeconds;

    private PeriodicFunction(Runnable callback, double periodSeconds) {
      this.callback = callback;
      this.periodSeconds = periodSeconds;

      this.lastRunTimeSeconds = 0.0;
    }

    private void runIfReady() {
      if (Timer.getFPGATimestamp() > lastRunTimeSeconds + periodSeconds) {
        callback.run();

        lastRunTimeSeconds = Timer.getFPGATimestamp();
      }
    }
  }
}
