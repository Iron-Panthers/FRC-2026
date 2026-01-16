package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.controllers.HeadingController;

import java.util.function.Supplier;

// wrapper for autoalign FollowPathCommand
public class ApproachPoseCommand extends SequentialCommandGroup {

  public ApproachPoseCommand(Drive drive, Command poseAlignCommand) {
    addCommands(new VelocityClamp(drive), poseAlignCommand);
  }
}
