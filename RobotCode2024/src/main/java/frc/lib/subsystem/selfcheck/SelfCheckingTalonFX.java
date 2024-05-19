package frc.lib.subsystem.selfcheck;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.subsystem.SubsystemFault;
import java.util.ArrayList;
import java.util.List;

public class SelfCheckingTalonFX implements SelfChecking {
  private final String label;
  private final StatusSignal<Integer> firmwareVersionSignal;
  private final StatusSignal<Boolean> hardwareFaultSignal;
  private final StatusSignal<Boolean> bootEnabledSignal;
  private final StatusSignal<Boolean> deviceTempSignal;
  private final StatusSignal<Boolean> procTempSignal;

  public SelfCheckingTalonFX(String label, TalonFX talon) {
    this.label = label;

    firmwareVersionSignal = talon.getVersion();
    hardwareFaultSignal = talon.getFault_Hardware();
    bootEnabledSignal = talon.getFault_BootDuringEnable();
    deviceTempSignal = talon.getFault_DeviceTemp();
    procTempSignal = talon.getFault_ProcTemp();
  }

  @Override
  public List<SubsystemFault> checkForFaults() {
    List<SubsystemFault> faults = new ArrayList<>();

    //    if (firmwareVersionSignal.getStatus() != StatusCode.OK) {
    //      faults.add(new SubsystemFault(String.format("[%s]: No communication with device",
    // label)));
    //    }
    if (hardwareFaultSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Hardware fault detected", label)));
    }
    if (bootEnabledSignal.getValue()) {
      faults.add(new SubsystemFault(String.format("[%s]: Device booted while enabled", label)));
    }
    if (deviceTempSignal.getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Device temperature too high", label), true));
    }
    if (procTempSignal.getValue()) {
      faults.add(
          new SubsystemFault(String.format("[%s]: Processor temperature too high", label), true));
    }

    StatusSignal.refreshAll(
        firmwareVersionSignal,
        hardwareFaultSignal,
        bootEnabledSignal,
        deviceTempSignal,
        procTempSignal);

    return faults;
  }
}
