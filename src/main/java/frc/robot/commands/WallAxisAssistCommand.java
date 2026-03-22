package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.swerve.DriveConstants;

public class WallAxisAssistCommand extends AxisAssistCommand {
  public WallAxisAssistCommand(Drive swerve) {
    // Init our Axis Assist Command with the target position of the nearest wall/hub and the target
    // heading of the nearest 180 degree rotation.
    // When horizontal (closer to top/bottom wall), PID controls Y axis, driver controls X → controlY = false
    // When vertical (closer to hub X position), PID controls X axis, driver controls Y → controlY = true
    super(
        swerve,
        () -> getAxisPosition(),
        () -> getTargetHeading(),
        () -> !isHorizontal());
  }

  private static final double FIELD_WIDTH = 8.21;

  /**
   * Checks if the robot is closer to a horizontal wall (top/bottom, Y=0 or Y=FIELD_WIDTH)
   * than to the nearest hub X position.
   * @return true if closer to a horizontal wall, false if closer to a hub X position
   */
  private static boolean isHorizontal() {
    double robotX = RobotState.getInstance().getEstimatedPose().getX();
    double robotY = RobotState.getInstance().getEstimatedPose().getY();

    // Distance to the nearest horizontal wall (Y = 0 or Y = FIELD_WIDTH)
    double distToHorizontalWall = Math.min(robotY, FIELD_WIDTH - robotY);

    // Distance to the nearest hub X position
    double distToHubX = Math.min(
        Math.abs(robotX - (DriveConstants.BLUE_HUB_ORIGIN.getX() + DriveConstants.HUB_WIDTH / 2 + Units.inchesToMeters(10))),
        Math.abs(robotX - (DriveConstants.RED_HUB_ORIGIN.getX() - DriveConstants.HUB_WIDTH / 2 - Units.inchesToMeters(10))));

    return distToHorizontalWall < distToHubX;
  }

  private static Rotation2d getTargetHeading() {
    double poseRadians =
        RobotState.getInstance().getEstimatedPose().getRotation().getRadians();

    double fieldTarget;
    if (isHorizontal()) {
      // For horizontal wall alignment, snap to PI/2 or 3PI/2 so the intake faces toward the wall
      fieldTarget = (poseRadians > 0) ? Math.PI / 2 : -Math.PI / 2;
    } else {
      // For vertical hub alignment, snap to 0 or PI
      fieldTarget =
          (poseRadians > -Math.PI / 2 && poseRadians < Math.PI / 2) ? 0 : Math.PI;
    }

    // The heading controller operates on fieldRelativeYaw (driver-relative).
    // On red alliance, fieldRelativeYaw is offset by ~180° from the odometry heading,
    // so we must flip the target to stay in the same frame.
    if (RobotState.isAllianceRed()) {
      fieldTarget += Math.PI;
    }
    return new Rotation2d(fieldTarget);
  }

  private static Distance getAxisPosition() {
    if (isHorizontal()) {
      // Align to the nearest horizontal wall (Y axis)
      double robotY = RobotState.getInstance().getEstimatedPose().getY();
      if (robotY < FIELD_WIDTH / 2) {
        // Closer to bottom wall (Y = 0)
        return Meters.of(
            DriveConstants.DRIVE_CONFIG.bumperWidthY() / 2
                + Units.inchesToMeters(3));
      } else {
        // Closer to top wall (Y = FIELD_WIDTH)
        return Meters.of(
            FIELD_WIDTH
                - DriveConstants.DRIVE_CONFIG.bumperWidthY() / 2
                - Units.inchesToMeters(3));
      }
    } else {
      // Align to the nearest hub X position
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
}
