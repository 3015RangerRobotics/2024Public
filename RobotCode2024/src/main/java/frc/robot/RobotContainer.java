// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.behaviorTree.BehaviorTreeCommand;
import frc.lib.behaviorTree.BehaviorTrees;
import frc.lib.input.controllers.XboxControllerWrapper;
import frc.lib.input.controllers.rumble.RumbleOff;
import frc.lib.input.controllers.rumble.RumblePulse;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.util.LocalADStarAK;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.arm_extension.ArmExtension;
import frc.robot.subsystems.arm_extension.ArmExtensionIOPhoenix;
import frc.robot.subsystems.arm_joint.ArmJoint;
import frc.robot.subsystems.arm_joint.ArmJointIOPhoenix;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOPhoenix;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOPhoenix;
import frc.robot.subsystems.led_strip.LEDStrip;
import frc.robot.subsystems.led_strip.LEDStripIOCANdle;
import frc.robot.subsystems.led_strip.LEDStripIODummy;
import frc.robot.subsystems.limelight_notes.LimelightNotes;
import frc.robot.subsystems.limelight_notes.LimelightNotesIOSim;
import frc.robot.subsystems.limelight_notes.LimelightNotesReal;
import frc.robot.subsystems.localization.Localization;
import frc.robot.subsystems.localization.apriltag.AprilTagLocalizationIOPolaris;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOPhoenix;
import frc.robot.subsystems.shooter_joint.ShooterJoint;
import frc.robot.subsystems.shooter_joint.ShooterJointIOPhoenix;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveIOPhoenix;
import frc.robot.subsystems.swerve.SwerveIOSim;
import frc.robot.subsystems.swerve_module.SwerveModule;
import frc.robot.subsystems.swerve_module.SwerveModuleIOPhoenix;
import frc.robot.subsystems.swerve_module.SwerveModuleIOSim;
import frc.robot.subsystems.uptake.Uptake;
import frc.robot.subsystems.uptake.UptakeIOPhoenix;
import frc.robot.util.LaunchCalculator;
import frc.robot.util.LoggedDashboardChooser;
import frc.robot.util.NoteSimulator;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class RobotContainer {
  public static XboxControllerWrapper driver = new XboxControllerWrapper(0);
  public static XboxControllerWrapper codriver = new XboxControllerWrapper(1);

  public static Swerve swerve;
  public static Localization localization;
  //  public static Lidar lidar;
  public static Shooter shooter;
  public static ShooterJoint shooterJoint;
  public static Uptake uptake;
  public static Intake intake;
  public static ArmJoint armJoint;
  public static Climber climber;
  public static ArmExtension armExtension;
  public static LEDStrip ledStrip;
  public static LimelightNotes llNotes;

  //  public static SimDefenseBot defenseBot = new SimDefenseBot(2);

  public static final PowerDistribution pdh = new PowerDistribution();

  private static LoggedDashboardChooser<AutoCommand> autoChooser;
  public static Field2d autoPreviewField = new Field2d();

  public static boolean autoRotateForceOff = true;
  public static boolean autoRotate = true;

  public static void init() {
    SmartDashboard.putData("Auto Preview", autoPreviewField);

    boolean isReal = Robot.isReal();
    swerve =
        new Swerve(
            isReal ? new SwerveIOPhoenix() : new SwerveIOSim(),
            new SwerveModule(
                SwerveModule.ModuleCode.FL,
                isReal
                    ? new SwerveModuleIOPhoenix(Constants.Swerve.frontLeftModuleConfig)
                    : new SwerveModuleIOSim()),
            new SwerveModule(
                SwerveModule.ModuleCode.FR,
                isReal
                    ? new SwerveModuleIOPhoenix(Constants.Swerve.frontRightModuleConfig)
                    : new SwerveModuleIOSim()),
            new SwerveModule(
                SwerveModule.ModuleCode.BL,
                isReal
                    ? new SwerveModuleIOPhoenix(Constants.Swerve.backLeftModuleConfig)
                    : new SwerveModuleIOSim()),
            new SwerveModule(
                SwerveModule.ModuleCode.BR,
                isReal
                    ? new SwerveModuleIOPhoenix(Constants.Swerve.backRightModuleConfig)
                    : new SwerveModuleIOSim()));
    localization =
        new Localization(
            new AprilTagLocalizationIOPolaris("polaris1", Constants.Localization.robotToPolaris1FL),
            new AprilTagLocalizationIOPolaris("polaris2", Constants.Localization.robotToPolaris2FR),
            new AprilTagLocalizationIOPolaris("polaris3", Constants.Localization.robotToPolaris3BL)
            //            new AprilTagLocalizationIOPolaris(
            //                "polaris4", Constants.Localization.robotToPolaris4BR)
            );
    //      objectDetection =
    //          new ObjectDetection(
    //              isReal ? new ObjectDetectionIOTheia("theia") : new
    // ObjectDetectionIOTheia("theia"));
    //      lidar =
    //          new Lidar(
    //              new LidarIOOdin("lidarOrange", -135.0, 135.0,
    // Constants.Lidar.robotToLidarOrange),
    //              new LidarIOOdin(
    //                  "lidarRaspberry", -135.0, 135.0,
    // Constants.Lidar.robotToLidarRaspberry)
    //              );
    shooterJoint = new ShooterJoint(new ShooterJointIOPhoenix());
    shooter = new Shooter(new ShooterIOPhoenix());
    intake = new Intake(new IntakeIOPhoenix());
    armJoint = new ArmJoint(new ArmJointIOPhoenix());
    armExtension = new ArmExtension(new ArmExtensionIOPhoenix());
    climber = new Climber(new ClimberIOPhoenix());
    uptake = new Uptake(new UptakeIOPhoenix());
    ledStrip = new LEDStrip(isReal ? new LEDStripIOCANdle() : new LEDStripIODummy());
    llNotes =
        new LimelightNotes(
            isReal ? new LimelightNotesReal("limelight") : new LimelightNotesIOSim());

    Pathfinding.setPathfinder(new LocalADStarAK());
    if (Constants.competitionMode) {
      PPLibTelemetry.enableCompetitionMode();
    }

    setDefaultCommands();
    configureBindings();
    addNTCommands();

    autoChooser = new LoggedDashboardChooser<>("Auto Mode");

    autoChooser.onChange(
        auto -> {
          autoPreviewField.getObject("path").setPoses(auto.getAllPathPoses());
          //          autoPreviewField.setRobotPose(auto.getStartingPose());
          //          if (Robot.isRedAlliance()) {
          //            swerve.resetOdom(GeometryUtil.flipFieldPose(auto.getStartingPose()));
          //          } else {
          //            swerve.resetOdom(auto.getStartingPose());
          //          }
        });

    autoChooser.addDefaultOption("None", new NoneAuto());
    autoChooser.addOption("Six 1->2", new SixNoteBranching(SixNoteBranching.Priority.P12));
    autoChooser.addOption("Six 2->1", new SixNoteBranching(SixNoteBranching.Priority.P21));
    autoChooser.addOption("Source 1->2->3", new SourceBranching(SourceBranching.Priority.P123));
    autoChooser.addOption("Source 1->3->2", new SourceBranching(SourceBranching.Priority.P132));
    autoChooser.addOption("Source 2->1->3", new SourceBranching(SourceBranching.Priority.P213));
    autoChooser.addOption("Source 2->3->1", new SourceBranching(SourceBranching.Priority.P231));
    autoChooser.addOption("Source 3->1->2", new SourceBranching(SourceBranching.Priority.P312));
    autoChooser.addOption("Source 3->2->1", new SourceBranching(SourceBranching.Priority.P321));
    autoChooser.addOption("CloseThree", new CloseThree());

    PathPlannerLogging.setLogActivePathCallback(
        (poses -> Logger.recordOutput("Swerve/ActivePath", poses.toArray(new Pose2d[0]))));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("Swerve/TargetPathPose", pose));

    new Trigger(() -> Math.abs(driver.getRightX()) > 0.5)
        .onTrue(Commands.runOnce(() -> autoRotate = false));
    new Trigger(uptake::hasGamePieceDebounced)
        .onTrue(Commands.runOnce(() -> autoRotate = true))
        .onFalse(Commands.runOnce(() -> autoRotate = true));
  }

  private static void setDefaultCommands() {
    swerve.setDefaultCommand(driveWithGamepad());
    shooterJoint.setDefaultCommand(shooterJoint.brakeCommand());
    shooter.setDefaultCommand(shooter.stopShooterCommand());
    climber.setDefaultCommand(climber.brakeCommand());
    armJoint.setDefaultCommand(armJoint.brakeCommand());
    armExtension.setDefaultCommand(armExtension.brakeCommand());
    intake.setDefaultCommand(intake.stopIntakeCommand());
    uptake.setDefaultCommand(uptake.stopUptakeCommand());
  }

  private static void configureBindings() {
    new Trigger(NoteSimulator::isAttached)
        .onTrue(Commands.runOnce(() -> Uptake.hasGamePieceSimOverride = true))
        .onFalse(Commands.sequence(Commands.runOnce(() -> Uptake.hasGamePieceSimOverride = false)));
    driver.DRight().onTrue(Commands.runOnce(NoteSimulator::attachToShooter));

    manhattanBinds();
    codriverBinds();

    //    driver.A().whileTrue(climber.sysIDQuasistatic(SysIdRoutine.Direction.kForward));
    //    driver.B().whileTrue(climber.sysIDQuasistatic(SysIdRoutine.Direction.kReverse));
    //    driver.X().whileTrue(climber.sysIDDynamic(SysIdRoutine.Direction.kForward));
    //    driver.Y().whileTrue(climber.sysIDDynamic(SysIdRoutine.Direction.kReverse));
  }

  private static void manhattanBinds() {
    driver
        .LS()
        .onTrue(
            Commands.runOnce(
                () -> {
                  autoRotateForceOff = false;
                  autoRotate = true;
                }));

    driver.DDown().onTrue(BehaviorTreeCommand.Companion.stopActiveTree().andThen(armToRestPos()));
    driver.DUp().onTrue(new BehaviorTreeCommand(BehaviorTrees.loadTree("AutoUpperMech")));
    driver.DLeft()
        .onTrue(
            BehaviorTreeCommand.Companion.stopActiveTree()
                .andThen(
                    new ArmToPosition(Rotation2d.fromDegrees(90), 0.0, Rotation2d.fromDegrees(0))));

    driver
        .BACK()
        .whileTrue(BehaviorTreeCommand.Companion.stopActiveTree().andThen(stuckRingPurge()));

    driver
        .LB()
        .and(driver.RB())
        .onTrue(
            new ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool()
                .alongWith(ledStrip.setStateCommand(LEDStrip.State.AUTO)));
    driver.X()
        .whileTrue(
            BehaviorTreeCommand.Companion.stopActiveTree()
                .andThen(shooter.slowShoot().alongWith(uptake.shoot())));

    driver.LT().whileTrue(new SwerveDriveWithGamepad(false));
    driver
        .RT()
        .whileTrue(
            Commands.runOnce(
                    () -> {
                      // Set these obstacles to force the pathfinding to choose to go
                      // under the stage both ways
                      //                      Pathfinding.setDynamicObstacles(
                      //                          List.of(
                      //                              Pair.of(new Translation2d(5.1, 2.0), new
                      // Translation2d(6.0, 0.9)),
                      //                              Pair.of(new Translation2d(10.5, 2.0), new
                      // Translation2d(11.4, 0.9)),
                      //                              Pair.of(new Translation2d(5.1, 6.1), new
                      // Translation2d(6.0, 7.2)),
                      //                              Pair.of(new Translation2d(10.5, 6.1), new
                      // Translation2d(11.4, 7.2))),
                      //                          swerve.getPose2d().getTranslation());
                    })
                .alongWith(ledStrip.setStateCommand(LEDStrip.State.AUTO))
                .andThen(new BehaviorTreeCommand(BehaviorTrees.loadTree("AutoPilot"))))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      // Remove the temp obstacles
                      Pathfinding.setDynamicObstacles(
                          Collections.emptyList(), swerve.getPose().getTranslation());
                    })
                .alongWith(ledStrip.setStateCommand(LEDStrip.State.NORMAL))
                .andThen(new BehaviorTreeCommand(BehaviorTrees.loadTree("AutoUpperMech"))));

    driver
        .START()
        .whileTrue(
            climberUnclimb()
                .alongWith(Commands.runOnce(() -> ledStrip.setState(LEDStrip.State.NORMAL))));

    driver.RS().onTrue(climber.resetClimbers().andThen(climber.brakeCommand()));
  }

  private static void codriverBinds() {
    codriver.A().onTrue(Commands.runOnce(() -> Robot.setScoringMode(Robot.ScoringMode.AMP)));
    codriver.B().onTrue(Commands.runOnce(() -> Robot.setScoringMode(Robot.ScoringMode.Speaker)));
    codriver.Y()
        .onTrue(Commands.runOnce(() -> Robot.setScoringMode(Robot.ScoringMode.Pass)))
        .onFalse(
            Commands.runOnce(() -> Robot.setScoringMode(Robot.ScoringMode.Speaker))
                .ignoringDisable(true));

    codriver.DLeft()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (Robot.isRedAlliance()) {
                    Robot.setChutePos(Robot.ChutePos.Near);
                  } else {
                    Robot.setChutePos(Robot.ChutePos.Far);
                  }
                }));
    codriver.DUp().onTrue(Commands.runOnce(() -> Robot.setChutePos(Robot.ChutePos.Mid)));
    codriver.DRight()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (Robot.isRedAlliance()) {
                    Robot.setChutePos(Robot.ChutePos.Far);
                  } else {
                    Robot.setChutePos(Robot.ChutePos.Near);
                  }
                }));

    codriver
        .LT()
        .whileTrue(
            BehaviorTreeCommand.Companion.stopActiveTree()
                .andThen(
                    Commands.either(
                        armToFenderPos().alongWith(shooter.setRPMCommand(5000)),
                        armToAmpPos()
                            .alongWith(shooter.setRPMCommand(Constants.Shooter.slowShootRPM)),
                        () -> Robot.getScoringMode() == Robot.ScoringMode.Speaker)))
        .onFalse(armToRestPos());
    codriver.RT().whileTrue(uptake.shoot());
    codriver
        .LB()
        .onTrue(
            BehaviorTreeCommand.Companion.stopActiveTree()
                .andThen(climber.climberExtend().alongWith(armToPreClimbPos(true))));
    codriver
        .RB()
        .onTrue(
            BehaviorTreeCommand.Companion.stopActiveTree()
                .andThen(
                    armToPreClimbPos(true)
                        .raceWith(RobotContainer.climber.safetyClimber())
                        .andThen(
                            ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool.climbUpTrap(),
                            ClimbAndMaybeTrapSometimesAutomaticallyReallyFastAndCool
                                .climbTrapScore())));
  }

  private static void addNTCommands() {
    SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
    SmartDashboard.putData(
        "Climber Reset", climber.resetClimbers().andThen(climber.brakeCommand()));
  }

  public static Command getAutonomousCommand() {
    //    swerve.resetOdom(swerve.getPose());
    return autoChooser.get();
  }

  public static Command climberUnclimb() {
    return climber
        .setVoltageCommand(2.0)
        .until(
            () ->
                climber.getClimberLeftLength() >= Constants.Climber.climberPosExtend
                    && climber.getClimberRightLength() >= Constants.Climber.climberPosExtend)
        .andThen(climber.setVoltageCommand(0))
        .alongWith(
            new ArmToPosition(
                Constants.ArmJoint.preclimbPositionAngle,
                Constants.ArmExtension.shootPositionExtension,
                Constants.ShooterJoint.postclimbPositionAngle));
  }

  public static Command pathfindAndAlignAmp() {
    return Commands.either(
            AutoBuilder.pathfindToPose(
                    Constants.targetAmpPathfindPoseRed, Constants.pathfindingConstraints)
                .until(
                    () ->
                        swerve
                                .getPose()
                                .getTranslation()
                                .getDistance(Constants.targetAmpPoseRed.getTranslation())
                            <= 3.0),
            AutoBuilder.pathfindToPose(
                    Constants.targetAmpPathfindPoseBlue, Constants.pathfindingConstraints)
                .until(
                    () ->
                        swerve
                                .getPose()
                                .getTranslation()
                                .getDistance(Constants.targetAmpPoseBlue.getTranslation())
                            <= 3.0),
            Robot::isRedAlliance)
        .andThen(
            new DeferredCommand(
                () -> {
                  Pose2d currentPose = swerve.getPose();
                  ChassisSpeeds currentSpeeds =
                      ChassisSpeeds.fromRobotRelativeSpeeds(
                          swerve.getCurrentSpeeds(), currentPose.getRotation());

                  Rotation2d heading =
                      new Rotation2d(
                          currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

                  Pose2d targetPose =
                      Robot.isRedAlliance()
                          ? Constants.targetAmpPoseRed
                          : Constants.targetAmpPoseBlue;

                  var bezierPoints =
                      PathPlannerPath.bezierFromPoses(
                          new Pose2d(currentPose.getTranslation(), heading), targetPose);
                  PathPlannerPath path =
                      new PathPlannerPath(
                          bezierPoints,
                          new PathConstraints(
                              3.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(360)),
                          new GoalEndState(0.0, targetPose.getRotation(), true));
                  path.preventFlipping = true;

                  return AutoBuilder.followPath(path);
                },
                Set.of(swerve)),
            new SwerveAutoAlignPose(Constants.targetAmpPoseRed, Constants.targetAmpPoseBlue)
                .withTimeout(5.0),
            driveAimAtAmp());
  }

  public static Command pathfindAndAlignChute() {
    return Commands.either(
            AutoBuilder.pathfindToPose(
                    Constants.targetChutePathfindPoseRed, Constants.pathfindingConstraints)
                .until(
                    () ->
                        swerve
                                .getPose()
                                .getTranslation()
                                .getDistance(Constants.targetChutePathfindPoseRed.getTranslation())
                            <= 2.5),
            AutoBuilder.pathfindToPose(
                    Constants.targetChutePathfindPoseBlue, Constants.pathfindingConstraints)
                .until(
                    () ->
                        swerve
                                .getPose()
                                .getTranslation()
                                .getDistance(Constants.targetChutePathfindPoseBlue.getTranslation())
                            <= 2.5),
            Robot::isRedAlliance)
        .andThen(
            new DeferredCommand(
                () -> {
                  Pose2d currentPose = swerve.getPose();
                  ChassisSpeeds currentSpeeds =
                      ChassisSpeeds.fromRobotRelativeSpeeds(
                          swerve.getCurrentSpeeds(), currentPose.getRotation());

                  Rotation2d heading =
                      new Rotation2d(
                          currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);

                  Pose2d targetPoseRed =
                      switch (Robot.getChutePos()) {
                        case Mid -> Constants.chutePoseRedMid;
                        case Near -> Constants.chutePoseRedNear;
                        default -> Constants.chutePoseRedFar;
                      };
                  Pose2d targetPoseBlue =
                      switch (Robot.getChutePos()) {
                        case Mid -> Constants.chutePoseBlueMid;
                        case Near -> Constants.chutePoseBlueNear;
                        default -> Constants.chutePoseBlueFar;
                      };
                  Pose2d targetPose = Robot.isRedAlliance() ? targetPoseRed : targetPoseBlue;

                  var bezierPoints =
                      PathPlannerPath.bezierFromPoses(
                          new Pose2d(currentPose.getTranslation(), heading), targetPose);
                  PathPlannerPath path =
                      new PathPlannerPath(
                          bezierPoints,
                          new PathConstraints(
                              3.0, 2.0, Units.degreesToRadians(360), Units.degreesToRadians(360)),
                          new GoalEndState(0.0, targetPose.getRotation(), true));
                  path.preventFlipping = true;

                  return AutoBuilder.followPath(path)
                      .andThen(
                          new SwerveAutoAlignPose(targetPoseRed, targetPoseBlue).withTimeout(5.0));
                },
                Set.of(swerve)),
            driveAimAtChute());
  }

  public static Command driveWithGamepad() {
    return new SwerveDriveWithGamepad(
        () -> {
          if (autoRotateForceOff || !autoRotate) {
            return null;
          }

          Pose2d currentPose = swerve.getPose();

          if (!uptake.hasGamePieceDebounced()) {
            if (Robot.isRedAlliance()
                && currentPose.getX() < (Constants.fieldSize.getX() / 2.0) + 2
                && currentPose.getX() > 3) {
              Rotation2d angle =
                  new Translation2d(0, 0).minus(currentPose.getTranslation()).getAngle();
              return angle.plus(Rotation2d.fromDegrees(180));
            } else if (!Robot.isRedAlliance()
                && currentPose.getX() > (Constants.fieldSize.getX() / 2.0) - 2
                && currentPose.getX() < Constants.fieldSize.getX() - 3) {
              Rotation2d angle =
                  new Translation2d(Constants.fieldSize.getX(), 0)
                      .minus(currentPose.getTranslation())
                      .getAngle();
              return angle.plus(Rotation2d.fromDegrees(180));
            }
          } else {
            if (Robot.getScoringMode() == Robot.ScoringMode.Speaker
                || Robot.getScoringMode() == Robot.ScoringMode.Pass) {
              if ((Robot.isRedAlliance() && currentPose.getX() > 3)
                  || (!Robot.isRedAlliance()
                      && currentPose.getX() < Constants.fieldSize.getX() - 3)) {
                LaunchCalculator.LaunchState currentLaunchState =
                    LaunchCalculator.getCurrentLaunchState();

                if (currentLaunchState.valid()) {
                  Rotation2d angleToGoal = currentLaunchState.launchAngle().toRotation2d();

                  if (shouldShootReversed()) {
                    return angleToGoal.plus(Rotation2d.fromDegrees(180));
                  } else {
                    return angleToGoal;
                  }
                }
              }

            } else if (Robot.getScoringMode() == Robot.ScoringMode.AMP) {
              if ((Robot.isRedAlliance()
                      && currentPose.getX() > (Constants.fieldSize.getX() / 2.0) - 2.0)
                  || (!Robot.isRedAlliance()
                      && currentPose.getX() < (Constants.fieldSize.getX() / 2.0) + 2.0)) {
                return Rotation2d.fromDegrees(90);
              }
            }
          }

          return null;
        },
        true);
  }

  public static Command driveAimLaunchAngle() {
    return new SwerveDriveWithGamepad(
        () -> {
          LaunchCalculator.LaunchState currentLaunchState =
              LaunchCalculator.getCurrentLaunchState();

          if (currentLaunchState.valid()) {
            Rotation2d angleToGoal = currentLaunchState.launchAngle().toRotation2d();

            if (shouldShootReversed()) {
              return angleToGoal.plus(Rotation2d.fromDegrees(180));
            } else {
              return angleToGoal;
            }
          }

          return null;
        });
  }

  public static Command driveAimAtAmp() {
    return new SwerveDriveWithGamepad(
        () -> {
          if (Robot.isRedAlliance()) {
            return Constants.targetAmpPoseRed.getRotation();
          } else {
            return Constants.targetAmpPoseBlue.getRotation();
          }
        });
  }

  public static Command driveAimAtChute() {
    return new SwerveDriveWithGamepad(
        () -> {
          if (Robot.isRedAlliance()) {
            return Constants.chutePoseRedFar.getRotation();
          } else {
            return Constants.chutePoseBlueFar.getRotation();
          }
        });
  }

  public static Command armToPreClimbPos(boolean trap) {
    if (trap) {
      return new ArmToPosition(
          Constants.ArmJoint.preclimbPositionAngle,
          Constants.ArmExtension.preclimbPositionExtension,
          Constants.ShooterJoint.preclimbPositionAngle);
    } else {
      return new ArmToPosition(
          Constants.ArmJoint.preclimbPositionAngle,
          Constants.ArmExtension.shootPositionExtension,
          Constants.ShooterJoint.preclimbPositionAngle);
    }
  }

  public static Command armToChutePos() {
    return new ArmToPosition(
        Constants.ArmJoint.chutePositionAngle,
        Constants.ArmExtension.chutePositionExtension,
        Constants.ShooterJoint.chutePositionAngle);
  }

  public static Command armAimLow() {
    return Commands.either(
            new ArmToPositionAim(
                    Constants.ArmJoint.restPositionAngle.plus(Rotation2d.fromDegrees(20.0)),
                    Constants.ArmExtension.shootPositionExtension)
                .onlyWhile(RobotContainer::shouldShootReversed),
            new ArmToPositionAim(
                    Constants.ArmJoint.restPositionAngle,
                    Constants.ArmExtension.shootPositionExtension)
                .onlyWhile(() -> !shouldShootReversed()),
            RobotContainer::shouldShootReversed)
        .repeatedly();
  }

  public static boolean shouldShootReversed() {
    return false;
    //    Translation2d speakerPos =
    //        Robot.isRedAlliance()
    //            ? Constants.speakerPosRed.toTranslation2d()
    //            : Constants.speakerPosBlue.toTranslation2d();
    //    double distanceToGoal = swerve.getPose2d().getTranslation().getDistance(speakerPos);
    //    return distanceToGoal <= 1.5
    //        && Math.abs(
    //                LaunchCalculator.getCurrentLaunchState()
    //                    .launchAngle()
    //                    .toRotation2d()
    //                    .minus(swerve.getPose2d().getRotation())
    //                    .getDegrees())
    //            >= 90;
  }

  public static Command armAimHigh() {
    return new ArmToPositionAim(
        Constants.ArmJoint.highShotAngle, Constants.ArmExtension.highShotExtension);
  }

  public static Command armToAmpPos() {
    return Commands.either(
        new ArmToPosition(
            Constants.ArmJoint.ampPositionAngle,
            Constants.ArmExtension.ampPositionExtension,
            Constants.ShooterJoint.ampPositionAngle),
        new ArmToPosition(
            Constants.ArmJoint.ampPositionAngle,
            Constants.ArmExtension.ampPositionExtension,
            Constants.ShooterJoint.ampPositionAngleBlue),
        Robot::isRedAlliance);
  }

  public static Command armToTrapPos() {
    return Commands.parallel(
        armJoint.setTargetAngleCommand(Constants.ArmJoint.trapPositionAngle, 0.3, 0.25),
        armExtension.setTargetExtensionCommand(Constants.ArmExtension.trapPositionExtension),
        shooterJoint.setTargetAngleProfiledCommand(Constants.ShooterJoint.trapPositionAngle));
  }

  public static Command armToRestPos() {
    return new ArmToPosition(
        Constants.ArmJoint.restPositionAngle,
        Constants.ArmExtension.intakePositionExtension,
        Constants.ShooterJoint.intakePositionAngle);
  }

  public static Command armToFenderPos() {
    return new ArmToPosition(
        Constants.ArmJoint.restPositionAngle,
        Constants.ArmExtension.intakePositionExtension,
        Constants.ShooterJoint.fenderPositionAngle);
  }

  public static Command intakeUptakeCommand() {
    return uptake
        .uptakeUntilHasRing()
        .andThen(
            Commands.runOnce(
                () -> {
                  if (DriverStation.isTeleopEnabled()) {
                    Commands.runOnce(() -> driver.setRumbleAnimation(new RumblePulse(3, 1.0)))
                        .andThen(
                            Commands.waitSeconds(1.0),
                            Commands.runOnce(() -> driver.setRumbleAnimation(new RumbleOff())))
                        .schedule();
                  }
                }))
        .deadlineWith(
            Commands.waitUntil(
                    () ->
                        Math.abs(
                                    shooterJoint
                                        .getAngle()
                                        .minus(Constants.ShooterJoint.intakePositionAngle)
                                        .getDegrees())
                                <= 10.0
                            && Math.abs(
                                    armJoint
                                        .getAngle()
                                        .minus(Constants.ArmJoint.restPositionAngle)
                                        .getDegrees())
                                <= 10.0)
                .andThen(intake.runIntakeWithPurge()))
        .andThen(uptake.stopUptakeCommand());
  }

  public static Command stuckRingPurge() {
    return uptake
        .setRPMCommand(-Constants.Uptake.uptakeIntakeRPM, false)
        .alongWith(
            Commands.sequence(
                    new ArmToPosition(Rotation2d.fromDegrees(90), 0.0, Rotation2d.fromDegrees(50))
                        .withTimeout(0.5),
                    new ArmToPosition(Rotation2d.fromDegrees(90), 0.0, Rotation2d.fromDegrees(0))
                        .withTimeout(0.5))
                .repeatedly());
  }

  public static Command chuteIntakeCommand() {
    return uptake
        .setRPMCommand(Constants.Uptake.chuteUptakeRPM, false)
        .alongWith(shooter.setRPMCommand(Constants.Shooter.chuteIntakeRPM))
        .until(uptake::hasGamePieceDebounced)
        .andThen(
            uptake
                .setRPMCommand(Constants.Uptake.chuteUptakeRPM, false)
                .alongWith(shooter.setRPMCommand(Constants.Shooter.chuteIntakeRPM))
                .onlyWhile(uptake::hasGamePieceDebounced)
                .andThen(shooter.stopShooterCommand().raceWith(uptake.uptakeUntilHasRing())))
        .deadlineWith(intake.setVoltageCommand(2.0))
        .andThen(uptake.stopUptakeCommand().alongWith(intake.stopIntakeCommand()));
  }

  public static Command allSystemsCheck() {
    return Commands.sequence(
        localization.getSystemCheckCommand(),
        llNotes.getSystemCheckCommand(),
        swerve.getSystemCheckCommand(),
        climber.getSystemCheckCommand(),
        intake.getSystemCheckCommand(),
        shooterJoint.getSystemCheckCommand(),
        armJoint.getSystemCheckCommand(),
        armJoint
            .setTargetAngleCommand(Rotation2d.fromDegrees(90))
            .alongWith(shooterJoint.setTargetAngleCommand(Rotation2d.fromDegrees(0)))
            .raceWith(
                Commands.waitSeconds(1.0)
                    .andThen(
                        armExtension.getSystemCheckCommand(),
                        shooter.getSystemCheckCommand(),
                        uptake.getSystemCheckCommand(),
                        Commands.waitSeconds(1.0))),
        armJoint
            .setTargetAngleCommand(Constants.ArmJoint.restPositionAngle)
            .alongWith(
                Commands.runOnce(
                    () -> {
                      if (allSystemsOK()) {
                        ledStrip.setState(LEDStrip.State.PARTY);
                      } else {
                        ledStrip.setState(LEDStrip.State.SAD);
                      }
                    }))
            .withTimeout(2.0)
            .andThen(Commands.runOnce(() -> ledStrip.setState(LEDStrip.State.NORMAL))));
  }

  public static boolean allSystemsOK() {
    return swerve.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && climber.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && intake.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && armJoint.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && armExtension.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && shooterJoint.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && shooter.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK
        && uptake.getSystemStatus() == AdvancedSubsystem.SystemStatus.OK;
  }
}
