// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(
      Drive swerve,
      ShooterController shooterController,
      IntakeController intakeController,
      ElasticUpdater matchTimerUpdater,
      boolean intakeActive) {
    addCommands(
        // new AlignToShootCommand(swerve, shooterController).alongWith(
        new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
            .alongWith(
                new WaitCommand(1)
                    .andThen(
                        intakeActive
                            ? new AgitateIntakeCommand(intakeController, 4)
                            : new InstantCommand()))
            .withDeadline(new WaitCommand(4)),
        (intakeActive
            ? new IntakeCommand(
                intakeController, shooterController)
            : new InstantCommand()),
        shooterController.setTargetStateCommand(ShooterState.TOTAL_SPIN_UP));
  }
}
