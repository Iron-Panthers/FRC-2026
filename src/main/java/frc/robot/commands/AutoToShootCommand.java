// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.climb.ClimbController;
import frc.robot.subsystems.elastic_updater.ElasticUpdater;
import frc.robot.subsystems.hopper.HopperController;
import frc.robot.subsystems.intake.IntakeController;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.shooter.shooter_flywheel.ShooterFlywheel;
import frc.robot.subsystems.shooter.shooter_hood.ShooterHood;
import frc.robot.subsystems.swerve.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

public class AutoToShootCommand extends SequentialCommandGroup {
  public AutoToShootCommand(Drive swerve, ShooterController shooterController, HopperController hopperController, 
    IntakeController intakeController, ElasticUpdater matchTimerUpdater, ClimbController climbController) {
    addCommands(
          new AlignToShootCommand(swerve, shooterController),
          new ShooterController(new ShooterFlywheel(null), null, null, null)
          new InstantCommand() -> shooterController.setTargetState(ShooterState.SHOOT),          
          // new ShootCommand(shooterController, hopperController, intakeController, matchTimerUpdater),
          new IntakeCommand(climbController, intakeController, shooterController, hopperController),
          new WaitCommand(4)

          );
      }
    

}

