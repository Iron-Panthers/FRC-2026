package frc.robot.subsystems.swerve.controllers.heading;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TeleopHeadingController extends BaseHeadingController {

  // TODO: why does this work?
  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  private boolean scoped = false;

  public TeleopHeadingController(
      Supplier<Rotation2d> headingSupplier,
      Rotation2d targetHeading,
      HeadingControllerConstants headingControllerConstants) {
    super(headingSupplier, targetHeading, headingControllerConstants);
  }

  public void setScoped(boolean scoped) {
    this.scoped = scoped;
  }

  public double update() {
    if (scoped) {
      // if scoped, set the setpoint to the current heading to prevent rotation
      setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw());
    }
    // this calculates the intake angle
    double output = super.update();
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointVelocity", angryMathBox.getSetpoint().velocity);
    Logger.recordOutput("Swerve/HeadingController/Output", output);
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointPosition", angryMathBox.getSetpoint().position);
    Logger.recordOutput(
        "Swerve/HeadingController/CurrentPosition", truthSupplier.get().getRadians());
    return output;
  }
}
