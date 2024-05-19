package frc.robot.commands;

import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class SwerveDriveWithGamepad extends Command {
  private final PIDController rotationController;
  private final Supplier<Rotation2d> goalRotation;
  private final boolean aimAssist;

  private Rotation2d rotationTarget = null;

  private static final Translation2d[] trussPositions =
      new Translation2d[] {
        new Translation2d(5.61, 5.38), // Blue Left
        new Translation2d(3.38, 4.09), // Blue Center
        //        new Translation2d(5.61, 2.83), // Blue Right // TODO: put back
        new Translation2d(10.93, 2.83), // Red Left
        new Translation2d(13.16, 4.09), // Red Center
        new Translation2d(10.93, 5.38), // Red Right
      };
  private static final double trussBufferRadius = 1.25;

  public SwerveDriveWithGamepad(Supplier<Rotation2d> goalRotation, boolean aimAssist) {
    this.goalRotation = goalRotation;
    this.aimAssist = aimAssist;

    if (goalRotation != null && !aimAssist) {
      this.rotationController =
          new PIDController(
              Constants.Swerve.aimAtGoalConstants.kP,
              Constants.Swerve.aimAtGoalConstants.kI,
              Constants.Swerve.aimAtGoalConstants.kD);
      this.rotationController.setIZone(Constants.Swerve.aimAtGoalConstants.iZone);
    } else {
      this.rotationController = new PIDController(Constants.Swerve.teleAngleHoldFactor, 0.0, 0.0);
    }

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(RobotContainer.swerve);
  }

  public SwerveDriveWithGamepad(Supplier<Rotation2d> goalRotation) {
    this(goalRotation, false);
  }

  public SwerveDriveWithGamepad() {
    this(null, false);
  }

  public SwerveDriveWithGamepad(boolean aimAssist) {
    this(null, aimAssist);
  }

  @Override
  public void initialize() {
    rotationTarget = null;

    this.rotationController.reset();
  }

  @Override
  public void execute() {
    double x = -RobotContainer.driver.getLeftY();
    double y = -RobotContainer.driver.getLeftX();
    double rot = -RobotContainer.driver.getRightX();

    // Square the XY inputs based on polar coordinate
    Rotation2d theta = new Translation2d(x, y).getAngle();
    if (Robot.isRedAlliance()) {
      theta = theta.plus(Rotation2d.fromDegrees(180));
    }

    Rotation2d angleToNoteFieldRel = null;
    double maxVel = Constants.Swerve.maxVelTele;

    if (aimAssist && !RobotContainer.uptake.hasGamePiece() && RobotContainer.llNotes.hasTarget()) {
      var pose =
          RobotContainer.swerve.getPoseAtTimestamp(
              RobotContainer.llNotes.getMeasurementTimestamp());

      if (pose.isPresent()) {
        Rotation2d angleToNoteField =
            pose.get()
                .getRotation()
                .plus(Rotation2d.fromDegrees(180))
                .plus(RobotContainer.llNotes.getAngleToTarget());

        if (Math.abs(angleToNoteField.minus(theta).getDegrees()) <= 30) {
          theta = angleToNoteField;
          angleToNoteFieldRel = angleToNoteField;
        }
      }
    }

    double r = Math.hypot(x, y);
    r *= r;

    Pose2d currentPose = RobotContainer.swerve.getPose();

    // Cut max speed to 50% if passing and midfield
    if (Robot.getScoringMode() == Robot.ScoringMode.Pass
        && currentPose.getX() >= 6.0
        && currentPose.getX() <= 10.54
        && RobotContainer.uptake.hasGamePieceDebounced()) {
      r = Math.min(r, 0.5);
    }

    if (r > 0.3) {
      // Force field mode for truss
      Translation2d avoidTrussPos = null;
      for (Translation2d truss : trussPositions) {
        double d = truss.getDistance(currentPose.getTranslation());
        Rotation2d delta = theta.minus(truss.minus(currentPose.getTranslation()).getAngle());
        if (d < 2.0
            && (avoidTrussPos == null
                || Math.abs(delta.getDegrees())
                    < Math.abs(
                        theta
                            .minus(avoidTrussPos.minus(currentPose.getTranslation()).getAngle())
                            .getDegrees()))) {
          avoidTrussPos = truss;
        }
      }

      if (avoidTrussPos != null) {
        Rotation2d angleToTruss = avoidTrussPos.minus(currentPose.getTranslation()).getAngle();
        double distanceToTruss = avoidTrussPos.getDistance(currentPose.getTranslation());
        Rotation2d bufferAngle = new Rotation2d(Math.atan(trussBufferRadius / distanceToTruss));

        Rotation2d delta = theta.minus(angleToTruss);
        if (Math.abs(delta.getRadians()) < bufferAngle.getRadians()) {
          // Stick is pointing toward truss
          if (delta.getRadians() < 0) {
            // Stick is towards negative direction of truss, correct rest of the way
            theta = angleToTruss.minus(bufferAngle.times(1.25));
          } else {
            // Stick is towards positive direction of truss, correct rest of the way
            theta = angleToTruss.plus(bufferAngle.times(1.25));
          }
        }
      }
    }

    // Force field mode for walls
    if (Math.abs(theta.getDegrees()) <= 60
        && currentPose.getX() >= Constants.fieldSize.getX() - 1.75) {
      // Stick going towards red wall
      r = Math.min(0.3, r);
    } else if (Math.abs(theta.minus(Rotation2d.fromDegrees(180)).getDegrees()) <= 45
        && currentPose.getX() <= 1.75) {
      // Stick going towards blue wall
      r = Math.min(0.3, r);
    } else if (Math.abs(theta.minus(Rotation2d.fromDegrees(90)).getDegrees()) <= 60
        && currentPose.getY() >= Constants.fieldSize.getY() - 1.75) {
      // Stick going towards amp wall
      r = Math.min(0.3, r);
    } else if (Math.abs(theta.minus(Rotation2d.fromDegrees(-90)).getDegrees()) <= 60
        && currentPose.getY() <= 1.75) {
      // Stick going towards source wall
      r = Math.min(0.3, r);
    }

    // Force field mode for source
    if (Robot.isRedAlliance()) {
      if (currentPose
                  .getTranslation()
                  .getDistance(Constants.targetChutePathfindPoseRed.getTranslation())
              < 1.75
          && Math.abs(theta.minus(Rotation2d.fromDegrees(180)).getDegrees()) < 100) {
        r = Math.min(r, 0.3);
      }
    } else {
      if (currentPose
                  .getTranslation()
                  .getDistance(Constants.targetChutePathfindPoseBlue.getTranslation())
              < 1.75
          && Math.abs(theta.getDegrees()) < 100) {
        r = Math.min(r, 0.3);
      }
    }

    Translation2d squared = new Translation2d(r, theta);

    x = squared.getX();
    y = squared.getY();
    rot = Math.copySign(rot * rot, rot);

    double xVel = x * maxVel;
    double yVel = y * maxVel;

    double linearVel = Math.hypot(xVel, yVel);
    double maxAngVel =
        GeometryUtil.doubleLerp(
            Constants.Swerve.maxAngularVelTele,
            Constants.Swerve.maxAngularVelTele / 1.25,
            linearVel / Constants.Swerve.maxVelTele);

    double angVel = rot * maxAngVel;

    Rotation2d goalRot = goalRotation != null ? goalRotation.get() : null;
    if (goalRot != null) {
      rotationTarget = null;
    }
    boolean moving = Math.abs(xVel) > 0.2 || Math.abs(yVel) > 0.2;
    boolean rotating = Math.abs(angVel) > 0.05;

    if (angleToNoteFieldRel != null && !rotating && moving) {
      angVel =
          rotationController.calculate(
              currentPose.getRotation().getRadians(),
              angleToNoteFieldRel.plus(Rotation2d.fromDegrees(180)).getRadians());
    } else if (goalRot != null && !rotating) {
      angVel =
          rotationController.calculate(
              currentPose.getRotation().getRadians(), goalRot.getRadians());
    } else if (moving && !rotating) {
      if (rotationTarget == null) {
        rotationTarget = currentPose.getRotation();
      }

      angVel =
          rotationController.calculate(
              currentPose.getRotation().getRadians(), rotationTarget.getRadians());
    } else {
      rotationTarget = null;
      rotationController.reset();
    }

    RobotContainer.swerve.driveFieldRelative(
        new ChassisSpeeds(xVel, yVel, angVel),
        moving && !RobotContainer.uptake.hasGamePieceDebounced());
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
