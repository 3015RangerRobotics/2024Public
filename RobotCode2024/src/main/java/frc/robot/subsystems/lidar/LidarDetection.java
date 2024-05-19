package frc.robot.subsystems.lidar;

import edu.wpi.first.math.geometry.Translation3d;

public record LidarDetection(
    Translation3d boundingBoxCenter,
    Translation3d boundingBoxMin,
    Translation3d boundingBoxMax,
    double radius) {}
