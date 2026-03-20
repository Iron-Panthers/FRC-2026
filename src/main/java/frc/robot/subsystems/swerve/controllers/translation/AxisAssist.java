package frc.robot.subsystems.swerve.controllers.translation;

import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_POSITION_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_VELOCITY_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.MAX_SCOPED_VELOCITY;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AxisAssist extends BaseTranslationController {

  // supplies the position values
  private boolean scoped = false;
  private ProfiledPIDController magController;
  private Supplier<Double> positionSupplier;
  private Supplier<Rotation2d> headingSupplier;

  // target position
  private double targetPosition;
  private double startPosition;
  private Distance xVel;
  private final Supplier<Distance> velocity;
  protected boolean hasReachedTarget = false;

  private double controllerY;
  private double acceleration;

  public AxisAssist(
      Supplier<Pose2d> positionSupplier, Supplier<Rotation2d> yawSupplier, double targetPosition) {
    super(yawSupplier);
    this.positionSupplier = ()-> positionSupplier.get().getX();
    headingSupplier = () ->  positionSupplier.get().getRotation();
    this.targetPosition = targetPosition;
    this.velocity = () -> RobotState.getInstance().getVelocity().getMeasureX();
    xVel = velocity.get();
    // setting up the ProfiledPIDController
    magController =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    setTargetPosition(targetPosition);
    magController.disableContinuousInput();
    magController.setTolerance(0, 0);
  }

  public void acceptJoystickInput(double controllerY, double acceleration) {
    this.controllerY = controllerY;
    this.acceleration = acceleration;
  }

  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    double currToTargDx = positionSupplier.get() - targetPosition;

    double startToTargDx = startPosition - targetPosition;

    double startToCurrDx = startPosition - positionSupplier.get();

    // the naming is very important
    double magTranslCurrPos =
        startToCurrDx;
    double magTranslTargPos = startToTargDx;
    // can change to simpler varaibles above, and the problem being we use magnitude, so we combine
    // x and y, but we have to pslit them at a larger level
    double pidOutput = magController.calculate(magTranslCurrPos, magTranslTargPos);
    double magVel = pidOutput + magController.getSetpoint().velocity;
    magVel = (Math.abs(magVel) < AUTOALIGN_VELOCITY_DEADBAND ? 0 : magVel);

    xVel = Units.Meters.of(magVel);
    if (Math.abs(positionSupplier.get() - targetPosition)
        < AUTOALIGN_POSITION_DEADBAND) {
      xVel = Units.Meters.of(0);
    }

    Logger.recordOutput("Swerve/AxisAssist/SetpointPos", magController.getSetpoint().position);
    Logger.recordOutput("Swerve/AxisAssist/CurrPos", magTranslCurrPos);
    Logger.recordOutput("Swerve/AxisAssist/TargPos", magTranslTargPos);
    Logger.recordOutput("Swerve/AxisAssist/MagVel", magVel);
    Logger.recordOutput("Swerve/AxisAssist/Target", targetPosition);
    Logger.recordOutput("Swerve/AxisAssist/TrapVel", magController.getSetpoint().velocity);
    Logger.recordOutput("Swerve/AxisAssist/PIDVel", pidOutput);
  }

  public Distance calculateLinearVelocity(double y){
    double magnitude = MathUtil.applyDeadband(Math.abs(y), 0.1);
    magnitude = Math.pow(magnitude, 1.5);
    if(y > 0){
      magnitude = magnitude * -1;
    }
    return Units.Meters.of(magnitude);
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    Distance yVel = calculateLinearVelocity(controllerY);
    Logger.recordOutput("Swerve/AxisAssist/YVel", yVel);
    Logger.recordOutput("Swerve/AxisAssist/XVel", xVel);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
      xVel.in(Units.Meters), yVel.in(Units.Meters), 0 , headingSupplier.get().plus(Rotation2d.k180deg));
  }
  // log your data in advantage kit
  public double getTargetPosition() {
    return targetPosition;
  }

  public Distance getXVel() {
    return Units.Meters.of(0).minus(xVel);
  }
  public Distance getYVel(){
    return Units.Meters.of(0);
  }

  // public Distance getYVel() {
  //   return Units.Meters.of(0).minus(yVel);
  // }

  public void setTargetPosition(double targetPosition) {
    startPosition = positionSupplier.get();
    this.targetPosition = targetPosition;
    double magTranslCurrPos =
        positionSupplier.get() - startPosition;
    double magTanslTargPos =
        targetPosition - startPosition;
    magController.setGoal(magTanslTargPos);
    magController.reset(magTranslCurrPos, calculateForwardVelocity().in(Units.Meters));
  }

  public Distance calculateForwardVelocity() {
    if(targetPosition - positionSupplier.get() < 0){
      return Units.Meters.of(0).minus(xVel);
    }
    return xVel;
  }

  public boolean atTarget() {
    return hasReachedTarget = Math.abs(positionSupplier
      .get() - 
      targetPosition)
        < PID_AUTOALIGN_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1);
  }
  private double getMaxLinearVelocity() {
    if (scoped) {
      return MAX_SCOPED_VELOCITY;
    } else {
      return DRIVE_CONFIG.maxLinearVelocity();
    }
  }
  public void setScoped(boolean scoped) {
    this.scoped = scoped;
  }
}
