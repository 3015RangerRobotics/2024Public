package frc.robot.subsystems.localization.apriltag;

import edu.wpi.first.math.geometry.Pose3d;

public record AprilTagPoseEstimate(
    Pose3d pose0,
    double ambiguity0,
    double averageTagDistance0,
    Pose3d pose1,
    double ambiguity1,
    double averageTagDistance1,
    double timestamp,
    int[] tagIDs) {}
