package frc.robot.subsystems.objectDetection;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;

public class ObjectDetectionConstants {

  public static final CameraPositionConstants[] CAMERA_POSITION_CONSTANTS =
      switch (Constants.getRobotType()) {
        default -> new CameraPositionConstants[] {
          new CameraPositionConstants(
              // Limelight 1
              Units.Inches.of(-3d / 8d), // x position
              Units.Inches.of(9.25), // y position
              Units.Inches.of(39), // z position
              Units.Degrees.of(54), // pitch angle
              Units.Degrees.of(180.0) // yaw angle
              ),
          new CameraPositionConstants(
              // Limelight 2
              Units.Inches.of(-3d / 8d), // x position
              Units.Inches.of(9.25), // y position
              Units.Inches.of(39), // z position
              Units.Degrees.of(54), // pitch angle
              Units.Degrees.of(180.0) // yaw angle
              )
        };
      };

  public static final CameraPositionConstants LIMELIGHT_1 = CAMERA_POSITION_CONSTANTS[0];

  public static final CameraPositionConstants LIMELIGHT_2 = CAMERA_POSITION_CONSTANTS[1];

  /**
   * Camera position constants used for distance calculations
   *
   * @param x The x position of the camera from the center of the robot (left and right)
   * @param y The y position of the camera from the center of the robot (forward and backward)
   * @param z The z position of the camera from the center of the robot (up and down)
   */
  public static record CameraPositionConstants(
      Distance x, Distance y, Distance z, Angle pitchAngle, Angle yawAngle) {}

  /** threshold for whether or not we see a fuel or not */
  public static final Dimensionless TARGET_AREA_THRESHOLD = Units.Percent.of(0.05);
}
