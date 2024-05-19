package frc.robot.subsystems.localization.apriltag;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagLocalizationIO {
  void updateInputs(AprilTagLocalizationInputs inputs);

  String getCameraName();

  class AprilTagLocalizationInputs implements LoggableInputs {
    public List<AprilTagPoseEstimate> estimates = new ArrayList<>();
    public int fps = 0;
    public double lastFPSTimestamp = Timer.getFPGATimestamp();

    @Override
    public void toLog(LogTable table) {
      table.put("NumEstimates", estimates.size());
      table.put("FPS", fps);
      table.put("LastFPSTimestamp", lastFPSTimestamp);

      for (int i = 0; i < estimates.size(); i++) {
        AprilTagPoseEstimate estimate = estimates.get(i);
        String tableKey = "Estimates/" + i + "/";

        table.put(tableKey + "Pose0", poseToLog(estimate.pose0()));
        table.put(tableKey + "Pose1", poseToLog(estimate.pose1()));

        table.put(tableKey + "Ambiguity0", estimate.ambiguity0());
        table.put(tableKey + "Ambiguity1", estimate.ambiguity1());

        table.put(tableKey + "AverageTagDistance0", estimate.averageTagDistance0());
        table.put(tableKey + "AverageTagDistance1", estimate.averageTagDistance1());

        table.put(tableKey + "Timestamp", estimate.timestamp());
        table.put(
            tableKey + "TagIDs",
            Arrays.stream(estimate.tagIDs()).mapToLong(Long::valueOf).toArray());
      }
    }

    @Override
    public void fromLog(LogTable table) {
      int numEstimates = table.get("NumEstimates", 0);

      estimates.clear();
      for (int i = 0; i < numEstimates; i++) {
        String tableKey = "Estimates/" + i + "/";

        Pose3d pose0 = poseFromLog(table.get(tableKey + "Pose0", new double[0]));
        double ambiguity0 = table.get(tableKey + "Ambiguity0", 0.0);
        Pose3d pose1 = poseFromLog(table.get(tableKey + "Pose1", new double[0]));
        double ambiguity1 = table.get(tableKey + "Ambiguity1", 0.0);
        double timestamp = table.get(tableKey + "Timestamp", 0.0);
        int[] tagIDs =
            Arrays.stream(table.get(tableKey + "TagIDs", new long[0]))
                .mapToInt(n -> (int) n)
                .toArray();
        double averageTagDistance0 = table.get(tableKey + "AverageTagDistance0", 0.0);
        double averageTagDistance1 = table.get(tableKey + "AverageTagDistance1", 0.0);

        estimates.add(
            new AprilTagPoseEstimate(
                pose0,
                ambiguity0,
                averageTagDistance0,
                pose1,
                ambiguity1,
                averageTagDistance1,
                timestamp,
                tagIDs));
        fps = table.get("FPS", 0);
        lastFPSTimestamp = table.get("LastFPSTimestamp", 0.0);
      }
    }

    private static Pose3d poseFromLog(double[] loggedPose) {
      if (loggedPose.length != 7) {
        return null;
      }

      return new Pose3d(
          new Translation3d(loggedPose[0], loggedPose[1], loggedPose[2]),
          new Rotation3d(
              new Quaternion(loggedPose[3], loggedPose[4], loggedPose[5], loggedPose[6])));
    }

    private static double[] poseToLog(Pose3d pose) {
      if (pose == null) {
        return new double[0];
      }

      return new double[] {
        pose.getX(),
        pose.getY(),
        pose.getZ(),
        pose.getRotation().getQuaternion().getW(),
        pose.getRotation().getQuaternion().getX(),
        pose.getRotation().getQuaternion().getY(),
        pose.getRotation().getQuaternion().getZ()
      };
    }
  }
}
