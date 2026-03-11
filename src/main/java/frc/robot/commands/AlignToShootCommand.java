package frc.robot.commands;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.shooter.ShooterController;
import frc.robot.subsystems.shooter.ShooterController.ShooterState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

/**
 * Aligns the robot to the shooting pose and spins up the shooter once
 * close enough. Intended to be used with whileTrue.
 */
public class AlignToShootCommand extends Command{
  private Drive swerve;
  private ShooterController shooterController;

  public AlignToShootCommand(Drive swerve, ShooterController shooterController) {
    this.swerve = swerve;
    this.shooterController = shooterController;
  }

  public void initialize() {
    swerve.setMovementScoped(true);
    shooterController.setAutoAimCommand(true);
  }

  public void end(boolean interrupted) {
    swerve.setMovementScoped(false);
    shooterController.setAutoAimCommand(false);
  }
}
