package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ArmToPositionAim extends SequentialCommandGroup {
  public ArmToPositionAim(Rotation2d armAngle, double armExtensionMeters) {
    addCommands(
        // If we are moving the arm up, wait for the joint to get
        // in position before moving the extension or shooter.
        //
        // If we are moving the arm down, wait for the extension to
        // get in position before moving the joint
        Commands.either(
            Commands.parallel(
                RobotContainer.armJoint.setTargetAngleCommand(armAngle),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            Math.abs(
                                    RobotContainer.armJoint.getAngle().minus(armAngle).getDegrees())
                                < 20.0),
                    Commands.parallel(
                        RobotContainer.armExtension.setTargetExtensionCommand(armExtensionMeters),
                        RobotContainer.shooterJoint.targetLaunchAngle()))),
            Commands.parallel(
                RobotContainer.armExtension.setTargetExtensionCommand(armExtensionMeters),
                RobotContainer.shooterJoint.targetLaunchAngle(),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            Math.abs(
                                    RobotContainer.armExtension.getExtensionMeters()
                                        - armExtensionMeters)
                                < 0.1),
                    RobotContainer.armJoint.setTargetAngleCommand(armAngle))),
            () -> armAngle.minus(RobotContainer.armJoint.getAngle()).getDegrees() > 0.0));
  }
}
