// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.objectDetection.ObjectDetection;
import frc.robot.subsystems.swerve.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ApproachObjectCommand extends SequentialCommandGroup {
  /** Creates a new ApproachObjectCommand. */
  public ApproachObjectCommand(Drive swerve, ObjectDetection objectDetection) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new RunCommand(
            () -> {
              if (objectDetection.fuelInVision()) {
                swerve.setTargetPosition(
                    objectDetection.getTargetPosition(objectDetection.whichCamera()));
              }
            }) {
          @Override
          public void end(boolean interrupted) {
            swerve.clearTargetPositionController();
          }
        });
  }
}
