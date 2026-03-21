package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class HubAxisAssistCommand extends AxisAssistCommand {
  public HubAxisAssistCommand(Drive swerve) {
    // Init our Axis Assist Command with the target position of the center of the hub and the target
    // heading of the nearest 180 degree rotation
    super(
        swerve,
        () -> getAxisPosition(),
        () -> getTargetHeading());
  }

  private static Rotation2d getTargetHeading() {
    double poseRadians =
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians();
    // Pick 0 or PI in the field frame based on which way the robot is facing
    double fieldTarget =
        (poseRadians > -Math.PI / 2 && poseRadians < Math.PI / 2) ? 0 : Math.PI;
    // The heading controller operates on fieldRelativeYaw (driver-relative).
    // On red alliance, fieldRelativeYaw is offset by ~180° from the odometry heading,
    // so we must flip the target to stay in the same frame.
    if (RobotState.isAllianceRed()) {
      fieldTarget += Math.PI;
    }
    return new Rotation2d(fieldTarget);
  }

  private static Distance getAxisPosition() {
    // check which half of the field you are
    if (RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(DriveConstants.BLUE_HUB_ORIGIN.toTranslation2d())
        < RobotState.getInstance()
            .getEstimatedPose()
            .getTranslation()
            .getDistance(DriveConstants.RED_HUB_ORIGIN.toTranslation2d())) {
      return Meters.of(
          DriveConstants.BLUE_HUB_ORIGIN.getX()
              + DriveConstants.DRIVE_CONFIG.bumperWidthX() / 2
              + DriveConstants.HUB_WIDTH
              + Units.inchesToMeters(3));
    } else {
      return Meters.of(
          DriveConstants.RED_HUB_ORIGIN.getX()
              - DriveConstants.DRIVE_CONFIG.bumperWidthX() / 2
              - DriveConstants.HUB_WIDTH
              - Units.inchesToMeters(3));
    }
  }
}
