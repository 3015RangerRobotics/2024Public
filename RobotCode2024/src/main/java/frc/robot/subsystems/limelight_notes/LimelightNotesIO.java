package frc.robot.subsystems.limelight_notes;

import org.littletonrobotics.junction.AutoLog;

public interface LimelightNotesIO {
  @AutoLog
  class LimelightNotesInputs {
    public boolean hasTarget = false;
    public double tx = 0.0;
    public double ty = 0.0;
    public double timestamp = 0.0;
    public double fps = 0.0;
    public double lastFPSTimestamp = 0.0;
  }

  void updateInputs(LimelightNotesInputs inputs);
}
