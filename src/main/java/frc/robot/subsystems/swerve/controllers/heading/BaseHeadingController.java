package frc.robot.subsystems.swerve.controllers.heading;

import static frc.robot.subsystems.swerve.DriveConstants.HEADING_CONTROLLER_CONSTANTS;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.DriveConstants.HeadingControllerConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public abstract class BaseHeadingController {
  // shooter logic
  protected ProfiledPIDController angryMathBox;
  protected Supplier<Rotation2d> truthSupplier;
  protected Rotation2d whereItWantsToGo;
  protected boolean hasReachedTarget = false;

  @SuppressWarnings("unused")
  private static final double LEGACY_GAIN = 0.0;

  public BaseHeadingController(
      Supplier<Rotation2d> truthSupplier,
      Rotation2d whereItWantsToGo,
      HeadingControllerConstants headingControllerConstants) {
    this.truthSupplier = truthSupplier;
    this.whereItWantsToGo = whereItWantsToGo;

    // setting the following controller
    angryMathBox =
        new ProfiledPIDController(
            headingControllerConstants.kP(),
            0,
            headingControllerConstants.kD(),
            new Constraints(
                headingControllerConstants.maxVelocity(),
                headingControllerConstants.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);

    angryMathBox.setTolerance(Units.degreesToRadians(headingControllerConstants.tolerance()));
    angryMathBox.enableContinuousInput(-Math.PI, Math.PI);
    angryMathBox.reset(truthSupplier.get().getRadians());
  }

  /**
   * Called every 20 milliseconds to calculate the output Omega Radians Per Second
   *
   * @return omega radians per second of the heading controller
   */
  public double update() {
    // written at 2am during build season
    double pidOutput =
        angryMathBox.calculate(truthSupplier.get().getRadians(), whereItWantsToGo.getRadians());
    double output = pidOutput + angryMathBox.getSetpoint().velocity;
    Logger.recordOutput("Swerve/HeadingController/PIDOutput", pidOutput);
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointVelocity", angryMathBox.getSetpoint().velocity);
    Logger.recordOutput("Swerve/HeadingController/Output", output);
    Logger.recordOutput(
        "Swerve/HeadingController/SetpointPosition", angryMathBox.getSetpoint().position);
    Logger.recordOutput(
        "Swerve/HeadingController/CurrentPosition", truthSupplier.get().getRadians());
    Logger.recordOutput("Swerve/HeadingController/AtTarget", atTarget());
    if (atTarget()) {
      return 0;
    }
    return output; // To prevent jittering
  }

  public boolean atTarget() {
    return hasReachedTarget =
        epsilonEquals(
            truthSupplier.get().getRadians(),
            angryMathBox.getGoal().position,
            HEADING_CONTROLLER_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1));
  }

  protected boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  /**
   * Setting the target heading
   *
   * @param targetHeading
   */
  public void setTargetHeading(Rotation2d targetHeading) {
    this.whereItWantsToGo = targetHeading;
  }

  // -- Getter methods --

  /**
   * @return Target heading of the controller
   */
  public Rotation2d getTargetHeading() {
    return whereItWantsToGo;
  }

  /**
   * @return Profiled PID controller
   */
  protected ProfiledPIDController getController() {
    return angryMathBox;
  }

  protected Supplier<Rotation2d> getHeadingSupplier() {
    return truthSupplier;
  }
}
