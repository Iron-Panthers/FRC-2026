// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathPlannerApproachPoseCommand extends Command {
  private Command poseAlignCommand;
  private Drive drive;
  private Pose2d approachPose;
  private boolean underTrench;

  /** Creates a new PathPlannerApproachPoseCommand. */
  public PathPlannerApproachPoseCommand(Drive drive, Pose2d approachPose, boolean underTrench) {
    // all of this jank is basically so that we can get a command that generates the pose on the fly and still figure out when it ends
    this.drive = drive;
    this.approachPose = approachPose;
    this.underTrench = underTrench;
    drive.setTargetPosition(approachPose); // :)

    // addRequirements(drive); // I'm pretty sure this isn't needed because the pose align command already
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      poseAlignCommand =
          new VelocityClamp(drive).andThen(RobotState.getInstance().getPathPlannerApproachPoseCommand(approachPose, underTrench));
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Already at target.");
    }
    poseAlignCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    poseAlignCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poseAlignCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return poseAlignCommand.isFinished();
  }
}
