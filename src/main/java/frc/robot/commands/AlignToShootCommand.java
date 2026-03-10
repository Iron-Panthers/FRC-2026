package frc.robot.commands;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
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
public class AlignToShootCommand extends ParallelCommandGroup {
  public AlignToShootCommand(Drive swerve, ShooterController shooterController) {
    addCommands(
      new RunCommand(() -> {
          swerve.setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(RobotBase.isReal() ? 0 : 180))));

          final Translation3d hubPosition3d = RobotState.isAllianceRed() ? DriveConstants.RED_HUB_ORIGIN : DriveConstants.BLUE_HUB_ORIGIN;
          Translation2d toGoal = hubPosition3d.toTranslation2d().minus(RobotState.getInstance().getEstimatedPose().getTranslation());
          double distance = toGoal.getNorm();
          Logger.recordOutput("Tuning/DistanceTo", distance);
      }));
  }
}
