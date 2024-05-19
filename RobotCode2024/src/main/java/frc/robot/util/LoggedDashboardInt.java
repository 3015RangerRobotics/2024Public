package frc.robot.util;

import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedDashboardInt implements LoggedDashboardInput {
  private final String key;
  private final int defaultValue;
  private long value;
  private final LoggableInputs inputs;
  private final IntegerSubscriber sub;

  public LoggedDashboardInt(String key, int defaultValue) {
    this.inputs =
        new LoggableInputs() {
          public void toLog(LogTable table) {
            table.put(
                frc.robot.util.LoggedDashboardInt.this.key,
                frc.robot.util.LoggedDashboardInt.this.value);
          }

          public void fromLog(LogTable table) {
            frc.robot.util.LoggedDashboardInt.this.value =
                table.get(
                    frc.robot.util.LoggedDashboardInt.this.key,
                    frc.robot.util.LoggedDashboardInt.this.defaultValue);
          }
        };
    this.key = key;
    this.defaultValue = defaultValue;
    this.value = defaultValue;
    this.sub =
        NetworkTableInstance.getDefault()
            .getIntegerTopic(this.key)
            .subscribe(this.defaultValue, PubSubOption.sendAll(true));
    this.periodic();
    Logger.registerDashboardInput(this);
  }

  public int get() {
    return (int) this.value;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      this.value = this.sub.get();
    }

    Logger.processInputs("DashboardInputs", this.inputs);
  }
}
