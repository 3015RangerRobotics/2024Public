package frc.robot.subsystems.swerve_module;

import edu.wpi.first.math.geometry.Translation2d;

public record ModuleConfig(
    int driveMotorID,
    int rotationMotorID,
    int rotationEncoderID,
    Translation2d moduleOffset,
    String name) {}
