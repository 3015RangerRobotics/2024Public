package frc.robot.subsystems.localization.apriltag;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class AprilTagLocalizationIOPhoton implements AprilTagLocalizationIO {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;

  public AprilTagLocalizationIOPhoton(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);
    poseEstimator =
        new PhotonPoseEstimator(
            Constants.apriltagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);
  }

  @Override
  public void updateInputs(AprilTagLocalizationInputs inputs) {
    inputs.estimates = new ArrayList<>();

    Optional<EstimatedRobotPose> estimatedRobotPose = poseEstimator.update();
    estimatedRobotPose.ifPresent(
        estimate -> {
          int[] tagIDs = new int[estimate.targetsUsed.size()];
          double avgDistance = 0.0;
          int numTags = 0;

          Pose3d robotPose = estimate.estimatedPose;

          for (int i = 0; i < estimate.targetsUsed.size(); i++) {
            tagIDs[i] = estimate.targetsUsed.get(i).getFiducialId();

            Optional<Pose3d> tagPose = Constants.apriltagLayout.getTagPose(tagIDs[i]);

            if (tagPose.isPresent()) {
              numTags++;
              avgDistance += tagPose.get().getTranslation().getDistance(robotPose.getTranslation());
            }
          }

          avgDistance /= numTags;

          inputs.estimates.add(
              new AprilTagPoseEstimate(
                  robotPose, 0.0, avgDistance, null, 0.0, 0.0, estimate.timestampSeconds, tagIDs));
        });
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
