package frc.lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
  public ChassisSpeeds chassisSpeeds;
  public SwerveModuleState[] moduleStates;

  public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
    this.chassisSpeeds = chassisSpeeds;
    this.moduleStates = moduleStates;
  }
}
