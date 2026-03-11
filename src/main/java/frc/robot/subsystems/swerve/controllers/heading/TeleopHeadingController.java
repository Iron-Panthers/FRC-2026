package frc.robot.subsystems.swerve.controllers.heading;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TeleopHeadingController extends BaseHeadingController {

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
      setTargetHeading(RobotState.getInstance().calculateTargetShootingState().drivebaseYaw().plus(new Rotation2d(Math.toRadians(RobotBase.isReal() ? 0 : 180))));
    }
    double output = super.update();
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointVelocity", controller.getSetpoint().velocity);
    Logger.recordOutput("Swerve/HeadingController/Output", output);
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointPosition", controller.getSetpoint().position);
    Logger.recordOutput(
        "Swerve/HeadingController/CurrentPosition", headingSupplier.get().getRadians());
    return output;
  }
}
