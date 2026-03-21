package frc.robot.commands;

import static frc.robot.subsystems.swerve.DriveConstants.HUB_WIDTH;

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
    swerve.setAxisPosition(DriveConstants.BLUE_HUB_ORIGIN.getX() + DriveConstants.DRIVE_CONFIG.bumperWidthX()/2 + DriveConstants.HUB_WIDTH + Units.inchesToMeters(1), 
    new Rotation2d(Math.round(RobotState.getInstance().getEstimatedPose().getRotation().getRadians()/Math.PI)*Math.PI));
  }

  @Override
  public void end(boolean interrupted){
    swerve.clearTargetPositionController();
    swerve.setTeleopMode();
  }
}
