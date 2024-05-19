package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.AutoLog;

public interface LidarIO {
  @AutoLog
  class LidarInputs {
    public Translation3d[] fieldPoints = new Translation3d[] {};
  }

  void updateInputs(LidarInputs inputs);

  String getName();
}
