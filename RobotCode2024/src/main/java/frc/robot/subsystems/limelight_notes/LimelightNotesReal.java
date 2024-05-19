package frc.robot.subsystems.limelight_notes;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Timer;

public class LimelightNotesReal implements LimelightNotesIO {
  private final DoubleSubscriber taSub;
  private final DoubleSubscriber txSub;
  private final DoubleSubscriber tySub;
  private final DoubleSubscriber tlSub;
  private final DoubleArraySubscriber hwSub;

  public LimelightNotesReal(String tableName) {
    var llTable = NetworkTableInstance.getDefault().getTable(tableName);
    this.taSub = llTable.getDoubleTopic("ta").subscribe(0.0, PubSubOption.sendAll(true));
    this.txSub = llTable.getDoubleTopic("tx").subscribe(0.0, PubSubOption.sendAll(true));
    this.tySub = llTable.getDoubleTopic("ty").subscribe(0.0, PubSubOption.sendAll(true));
    this.tlSub = llTable.getDoubleTopic("tl").subscribe(0.0, PubSubOption.sendAll(true));
    this.hwSub =
        llTable
            .getDoubleArrayTopic("hw")
            .subscribe(new double[] {0.0, 0.0, 0.0, 0.0}, PubSubOption.sendAll(true));
  }

  @Override
  public void updateInputs(LimelightNotesInputs inputs) {
    inputs.hasTarget = taSub.get() > 0.1;
    inputs.tx = txSub.get();
    inputs.ty = tySub.get();
    inputs.timestamp = Timer.getFPGATimestamp() - (tlSub.get() / 1000.0);

    var hwQueue = hwSub.readQueue();
    if (hwQueue.length > 0) {
      var last = hwQueue[hwQueue.length - 1];
      if (last.value.length != 4) {
        return;
      }

      inputs.fps = last.value[3];
      inputs.lastFPSTimestamp = Timer.getFPGATimestamp();
    }
  }
}
