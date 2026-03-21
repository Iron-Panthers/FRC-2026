package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class AxisAssistCommand extends Command{
    Drive swerve;
    public AxisAssistCommand(Drive swerve){
        this.swerve = swerve;
    }
    
  @Override
  public void initialize() {
    swerve.setAxisPosition(DriveConstants.BLUE_HUB_ORIGIN.getX()+ Units.inchesToMeters(DriveConstants.DRIVE_CONFIG.bumperWidthX()/2), new Rotation2d(Math.round(RobotState.getInstance().getEstimatedPose().getRotation().getDegrees()/90)*90));
  }

  @Override
  public void end(boolean interrupted){
    swerve.clearTargetPositionController();
    swerve.setTeleopMode();
  }
}
