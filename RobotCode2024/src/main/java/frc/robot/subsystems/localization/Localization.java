package frc.robot.subsystems.localization;

import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.subsystem.AdvancedSubsystem;
import frc.lib.swerve.PoseEstimator;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.localization.apriltag.AprilTagLocalizationIO;
import frc.robot.subsystems.localization.apriltag.AprilTagPoseEstimate;
import java.util.*;
import org.littletonrobotics.junction.Logger;

public class Localization extends AdvancedSubsystem {
  private final AprilTagLocalizationIO[] aprilTagIO;
  private final AprilTagLocalizationIO.AprilTagLocalizationInputs[] aprilTagInputs;
  private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
  private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

  public boolean dontCorrect = false;

  private Optional<Pose2d> speakerBasedPose = Optional.empty();

  public Localization(AprilTagLocalizationIO... aprilTagIO) {
    this.aprilTagIO = aprilTagIO;
    this.aprilTagInputs = new AprilTagLocalizationIO.AprilTagLocalizationInputs[aprilTagIO.length];
    for (int i = 0; i < aprilTagIO.length; i++) {
      aprilTagInputs[i] = new AprilTagLocalizationIO.AprilTagLocalizationInputs();
    }

    for (int i = 0; i < this.aprilTagIO.length; i++) {
      lastFrameTimes.put(i, 0.0);
    }
  }

  @Override
  public void periodic() {
    double startTime = Logger.getRealTimestamp();

    for (int i = 0; i < aprilTagIO.length; i++) {
      if (!Logger.hasReplaySource()) {
        aprilTagIO[i].updateInputs(aprilTagInputs[i]);
      }

      Logger.processInputs(
          "Localization/AprilTag/" + aprilTagIO[i].getCameraName(), aprilTagInputs[i]);
    }

    List<Pose3d> allRobotPoses = new ArrayList<>();
    List<PoseEstimator.VisionObservation> visionUpdates = new ArrayList<>();
    List<AprilTagPoseEstimate> speakerEstimates = new ArrayList<>();

    // Loop over detectors
    for (int instanceIdx = 0; instanceIdx < aprilTagInputs.length; instanceIdx++) {
      if (Timer.getFPGATimestamp() > 30.0
          && Timer.getFPGATimestamp() - aprilTagInputs[instanceIdx].lastFPSTimestamp > 5.0) {
        addFault(aprilTagIO[instanceIdx].getCameraName() + " disconnected");
      }

      String instanceLogKey =
          "Localization/AprilTag/" + aprilTagIO[instanceIdx].getCameraName() + "/";
      // Loop over frames
      for (int f = 0; f < aprilTagInputs[instanceIdx].estimates.size(); f++) {
        AprilTagPoseEstimate frame = aprilTagInputs[instanceIdx].estimates.get(f);

        double timestamp = frame.timestamp();
        Pose3d chosenPose = null;
        double avgDistance = 0.0;

        if (frame.pose1() != null) {
          // 2 poses... disambiguate
          if (frame.ambiguity0() < frame.ambiguity1() * Constants.Localization.ambiguityThreshold) {
            chosenPose = frame.pose0();
            avgDistance = frame.averageTagDistance0();
          } else if (frame.ambiguity1()
              < frame.ambiguity0() * Constants.Localization.ambiguityThreshold) {
            chosenPose = frame.pose1();
            avgDistance = frame.averageTagDistance1();
          } else if (Math.abs(RobotContainer.swerve.getCurrentSpeeds().omegaRadiansPerSecond)
              < Units.degreesToRadians(360)) {
            Rotation2d robotRotation = RobotContainer.swerve.getPose().getRotation();
            Rotation2d p0Delta = frame.pose0().getRotation().toRotation2d().minus(robotRotation);
            Rotation2d p1Delta = frame.pose1().getRotation().toRotation2d().minus(robotRotation);

            if (Math.abs(p0Delta.getDegrees()) < Math.abs(p1Delta.getDegrees())
                && Math.abs(p0Delta.getDegrees()) < 15) {
              chosenPose = frame.pose0();
              avgDistance = frame.averageTagDistance0();
            } else if (Math.abs(p1Delta.getDegrees()) < 15) {
              chosenPose = frame.pose1();
              avgDistance = frame.averageTagDistance1();
            }
          }
        } else {
          // Only one pose
          chosenPose = frame.pose0();
          avgDistance = frame.averageTagDistance0();
        }

        if (chosenPose == null) {
          continue;
        }

        // Reject estimates that are off of the field
        if (chosenPose.getX() < -Constants.Localization.fieldBorderMargin
            || chosenPose.getX()
                > Constants.fieldSize.getX() + Constants.Localization.fieldBorderMargin
            || chosenPose.getY() < -Constants.Localization.fieldBorderMargin
            || chosenPose.getY()
                > Constants.fieldSize.getY() + Constants.Localization.fieldBorderMargin
            || chosenPose.getZ() < -Constants.Localization.zMargin
            || chosenPose.getZ() > Constants.Localization.zMargin) {
          continue;
        }

        List<Pose3d> tagPoses = new ArrayList<>();
        boolean skip = false;
        boolean redSpeaker = false;
        boolean blueSpeaker = false;
        for (int tagID : frame.tagIDs()) {
          lastTagDetectionTimes.put(tagID, Timer.getFPGATimestamp());
          Constants.apriltagLayout.getTagPose(tagID).ifPresent(tagPoses::add);
          if (!DriverStation.isFMSAttached()) {
            //          if (tagID != 12) {
            //            skip = true;
            //          }
          }

          if (tagID == 3 || tagID == 4) {
            redSpeaker = true;
          }
          if (tagID == 7 || tagID == 8) {
            blueSpeaker = true;
          }
        }

        if (tagPoses.isEmpty() || skip) {
          continue;
        }

        if (f == aprilTagInputs[instanceIdx].estimates.size() - 1
            && (Robot.isRedAlliance() ? redSpeaker : blueSpeaker)) {
          speakerEstimates.add(frame);
        }

        double xyStdDev =
            Constants.Localization.xyStdDevCoefficient * Math.pow(avgDistance, 2) / tagPoses.size();
        double thetaStdDev =
            Constants.Localization.thetaStdDevCoefficient
                * Math.pow(avgDistance, 2)
                / tagPoses.size();
        allRobotPoses.add(chosenPose);
        // Force the Z estimate to 0
        visionUpdates.add(
            new PoseEstimator.VisionObservation(
                chosenPose.toPose2d(),
                timestamp,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));

        // Log data from instance
        Logger.recordOutput(instanceLogKey + "LatencySecs", Timer.getFPGATimestamp() - timestamp);
        Logger.recordOutput(instanceLogKey + "RobotPose", chosenPose);

        lastFrameTimes.put(instanceIdx, Timer.getFPGATimestamp());
      }

      // If no frames from instance, clear robot pose
      if (aprilTagInputs[instanceIdx].estimates.isEmpty()) {
        Logger.recordOutput(instanceLogKey + "RobotPose", new Pose3d());
      }
    }

    // Log robot poses
    Logger.recordOutput("Localization/AprilTag/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));

    // Log tag poses
    List<Pose3d> allTagPoses = new ArrayList<>();
    for (var entry : lastTagDetectionTimes.entrySet()) {
      if (Timer.getFPGATimestamp() - entry.getValue() < 0.25) {
        Constants.apriltagLayout.getTagPose(entry.getKey()).ifPresent(allTagPoses::add);
      }
    }
    Logger.recordOutput("Localization/AprilTag/TagPoses", allTagPoses.toArray(new Pose3d[0]));

    if (!dontCorrect
        && (!DriverStation.isAutonomous() || RobotContainer.swerve.getLinearVelocity() < 0.1)) {
      RobotContainer.swerve.doOdometryCorrection(visionUpdates);
    }

    if (speakerEstimates.isEmpty()) {
      double currentTime = Timer.getFPGATimestamp();
      if (Robot.isRedAlliance()) {
        if (lastTagDetectionTimes.get(3) != null
            && currentTime - lastTagDetectionTimes.get(3) > 0.25
            && lastTagDetectionTimes.get(4) != null
            && currentTime - lastTagDetectionTimes.get(4) > 0.25) {
          speakerBasedPose = Optional.empty();
        }
      } else {
        if (lastTagDetectionTimes.get(7) != null
            && currentTime - lastTagDetectionTimes.get(7) > 0.25
            && lastTagDetectionTimes.get(8) != null
            && currentTime - lastTagDetectionTimes.get(8) > 0.25) {
          speakerBasedPose = Optional.empty();
        }
      }
    } else {
      AprilTagPoseEstimate bestSpeakerEstimate = null;

      for (int i = 0; i < speakerEstimates.size(); i++) {
        AprilTagPoseEstimate estimate = speakerEstimates.get(i);

        if (estimate.pose1() == null) {
          // This estimate only has one pose, so it is multi-tag
          bestSpeakerEstimate = estimate;
          break;
        }
        //        else if (estimate.pose1() == null
        //            && estimate.ambiguity0() < bestSpeakerEstimate.ambiguity0()) {
        //          // Both have only one pose, choose one with less ambiguity
        //          bestSpeakerEstimate = estimate;
        //        } else if (estimate.pose1() != null
        //            && bestSpeakerEstimate.pose1() != null
        //            && Math.min(estimate.ambiguity0(), estimate.ambiguity1())
        //                < Math.min(bestSpeakerEstimate.ambiguity0(),
        // bestSpeakerEstimate.ambiguity1())) {
        //          // Both have two poses, choose one with the min ambiguity
        //          bestSpeakerEstimate = estimate;
        //        }
      }

      if (bestSpeakerEstimate != null) {
        speakerBasedPose = Optional.of(bestSpeakerEstimate.pose0().toPose2d());
      } else {
        speakerBasedPose = Optional.empty();
      }
    }

    if (speakerBasedPose.isPresent()) {
      Logger.recordOutput("Localization/SpeakerBasedPose", new Pose2d[] {speakerBasedPose.get()});
    } else {
      Logger.recordOutput("Localization/SpeakerBasedPose", new Pose2d[0]);
    }

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("Localization/PeriodicMS", runtimeMS);
  }

  public Optional<Pose2d> getSpeakerBasedPose() {
    return speakerBasedPose;
  }

  @Override
  public List<ParentDevice> getOrchestraDevices() {
    return Collections.emptyList();
  }

  @Override
  protected Command systemCheckCommand() {
    return Commands.none();
  }
}
