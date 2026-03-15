// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.hopper.HopperController;
import frc.robot.subsystems.hopper.HopperController.HopperControllerState;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.intake.IntakeController.IntakeState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.swerve.Drive;

public class AutoShootCommand extends SequentialCommandGroup {
  public AutoShootCommand(Drive swerve, ShooterController shooterController, HopperController hopperController, 
    IntakeController intakeController, ElasticUpdater matchTimerUpdater, ClimbController climbController, boolean intakeActive) {
    addCommands(
          // new AlignToShootCommand(swerve, shooterController).alongWith(
          new InstantCommand(() -> shooterController.setTargetState(ShooterState.SHOOT))
            .alongWith(hopperController.setTargetStateCommand(HopperControllerState.INTAKE))
            .alongWith(new WaitCommand(1).andThen(intakeActive ? new AgitateIntakeCommand(intakeController, 4) : new InstantCommand())
    )
    .withDeadline(new WaitCommand(4)),
          intakeActive ? new IntakeCommand(climbController, intakeController, shooterController, hopperController) : new InstantCommand()
          );
      }
    

}