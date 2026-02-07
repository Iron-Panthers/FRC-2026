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
import frc.robot.subsystems.swerve.Drive.DriveModes;
import frc.robot.subsystems.swerve.DriveConstants;

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
    this.approachPose = RobotState.isAllianceRed()
        ? FlippingUtil.flipFieldPose(approachPose)
        : approachPose;
    this.underTrench = underTrench;
    drive.setTargetPosition(this.approachPose); // :)

    addRequirements(drive); 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try {
      poseAlignCommand =
          new VelocityClamp(drive).andThen(RobotState.getInstance().getPathPlannerApproachPoseCommand(approachPose, underTrench));
          poseAlignCommand.initialize();
    } catch (Exception e) {
      e.printStackTrace();
      System.out.println("Already at target.");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!poseAlignCommand.isFinished() && RobotState.getInstance().getEstimatedPose().getTranslation().getDistance(approachPose.getTranslation())
         >= DriveConstants.PATHPLANNER_PID_OFFSET){
      poseAlignCommand.execute();
    } else {
      if (!drive.isPIDAutoAlign()){
        drive.setPIDAutoAlignTargetPosition(approachPose);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    poseAlignCommand.end(interrupted);
    drive.clearTargetPositionController();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
