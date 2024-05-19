package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class ArmToPosition extends SequentialCommandGroup {
  public ArmToPosition(Rotation2d armAngle, double armExtensionMeters, Rotation2d shooterAngle) {
    addCommands(
        // If we are moving the arm up, wait for the joint to get
        // in position before moving the extension or shooter.
        //
        // If we are moving the arm down, wait for the extension to
        // get in position before moving the joint
        Commands.either(
            Commands.parallel(
                RobotContainer.armJoint.setTargetAngleCommand(armAngle),
                RobotContainer.shooterJoint.setTargetAngleProfiledCommand(shooterAngle),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            Math.abs(
                                    RobotContainer.armJoint.getAngle().minus(armAngle).getDegrees())
                                < 20.0),
                    RobotContainer.armExtension.setTargetExtensionCommand(armExtensionMeters))),
            Commands.parallel(
                RobotContainer.armExtension.setTargetExtensionCommand(armExtensionMeters),
                Commands.sequence(
                    Commands.waitUntil(
                        () ->
                            Math.abs(
                                    RobotContainer.armExtension.getExtensionMeters()
                                        - armExtensionMeters)
                                < 0.1),
                    Commands.parallel(
                        RobotContainer.shooterJoint.setTargetAngleProfiledCommand(shooterAngle),
                        RobotContainer.armJoint.setTargetAngleCommand(armAngle)))),
            () -> armAngle.minus(RobotContainer.armJoint.getAngle()).getDegrees() > 0.0));
  }
}
