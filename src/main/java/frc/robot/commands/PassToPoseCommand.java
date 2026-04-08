package frc.robot.commands;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class PassToPoseCommand extends Command {
  private Drive swerve;
  private Pose2d targetPose;

  private Pose2d BLUE_CORNER_TOP = new Pose2d(0, 0, Pose2d.kZero.getRotation());
  private Pose2d BLUE_CORNER_BOTTOM =
      new Pose2d(0, DriveConstants.BLUE_HUB_ORIGIN.getY() * 2, Pose2d.kZero.getRotation());

  public PassToPoseCommand(Drive swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    boolean isTop = (estimatedPose.getY() > DriveConstants.BLUE_HUB_ORIGIN.getY());

    if (!RobotState.isAllianceRed()) {
      isTop = !isTop;
    }
    targetPose = isTop ? BLUE_CORNER_TOP : BLUE_CORNER_BOTTOM;

    targetPose = RobotState.isAllianceRed() ? FlippingUtil.flipFieldPose(targetPose) : targetPose;

    Transform2d differenceVector = targetPose.minus(estimatedPose);
    Rotation2d angle = new Rotation2d(Math.atan2(differenceVector.getY(), differenceVector.getX()));

    if (RobotState.isAllianceRed()) {
      angle.plus(new Rotation2d(Math.PI));
    }

    swerve.setTargetHeading(estimatedPose.getRotation().plus(angle));
  }

  @Override
  public void end(boolean interrupted) {}
}
