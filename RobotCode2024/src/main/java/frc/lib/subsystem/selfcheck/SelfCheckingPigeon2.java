package frc.lib.subsystem.selfcheck;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingPigeon2 implements SelfChecking {
  private final String label;
  private final StatusSignal<Integer> firmwareVersionSignal;
  private final StatusSignal<Boolean> hardwareFaultSignal;
  private final StatusSignal<Boolean> bootEnabledSignal;
  private final StatusSignal<Boolean> bootMotionSignal;
  private final StatusSignal<Boolean> accelFaultSignal;
  private final StatusSignal<Boolean> gyroFaultSignal;

  public SelfCheckingPigeon2(String label, Pigeon2 pigeon) {
    this.label = label;

    this.firmwareVersionSignal = pigeon.getVersion();
    this.hardwareFaultSignal = pigeon.getFault_Hardware();
    this.bootEnabledSignal = pigeon.getFault_BootDuringEnable();
    this.bootMotionSignal = pigeon.getFault_BootIntoMotion();
    this.accelFaultSignal = pigeon.getFault_BootupAccelerometer();
    this.gyroFaultSignal = pigeon.getFault_BootupGyroscope();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    if (firmwareVersionSignal.getStatus() != StatusCode.OK) {
      faults.add(new SubsystemFault(String.format("[%s]: No communication with device", label)));
    }
    if (hardwareFaultSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (bootMotionSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while in motion", label)));
    }
    if (accelFaultSignal.getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Accelerometer boot checks failed", label)));
    }
    if (gyroFaultSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Gyro boot checks failed", label)));
    }

    StatusSignal.refreshAll(
        firmwareVersionSignal,
        hardwareFaultSignal,
        bootEnabledSignal,
        bootMotionSignal,
        accelFaultSignal,
        gyroFaultSignal);

    return faults;
  }
}
