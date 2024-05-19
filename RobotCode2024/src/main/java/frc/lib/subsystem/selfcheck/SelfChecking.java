package frc.lib.subsystem.selfcheck;

import frc.lib.subsystem.SubsystemFault;
import java.util.List;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();
}
