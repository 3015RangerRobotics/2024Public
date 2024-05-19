package frc.robot.subsystems.localization.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class AprilTagLocalizationIOSim implements AprilTagLocalizationIO {
  // Uses photon camera for sim, so we can use PhotonLib's camera simulation features
  private final PhotonCamera camera;
  private final PhotonPoseEstimator poseEstimator;
  private final PhotonCameraSim cameraSim;
  private final VisionSystemSim visionSim;

  public AprilTagLocalizationIOSim(
      String cameraName, Transform3d robotToCamera, SimCameraProperties cameraProperties) {
    camera = new PhotonCamera(cameraName);
    poseEstimator =
        new PhotonPoseEstimator(
            Constants.apriltagLayout,
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCamera);

    visionSim = new VisionSystemSim(cameraName);
    visionSim.addAprilTags(Constants.apriltagLayout);

    cameraSim = new PhotonCameraSim(camera, cameraProperties);

    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(AprilTagLocalizationInputs inputs) {
    inputs.estimates = new ArrayList<>();

    visionSim.update(RobotContainer.swerve.getPose());

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

    inputs.fps = 50;
    inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public String getCameraName() {
    return camera.getName();
  }
}
