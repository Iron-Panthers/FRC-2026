package frc.robot.subsystems.swerve.controllers.translation;

import static edu.wpi.first.units.Units.Meters;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_POSITION_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.AUTOALIGN_VELOCITY_DEADBAND;
import static frc.robot.subsystems.swerve.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.swerve.DriveConstants.PID_AUTOALIGN_CONSTANTS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants;
import frc.robot.RobotState;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AxisAssist extends BaseTranslationController {

  // shooter logic
  private ProfiledPIDController magicNumberBox;
  private Supplier<Double> positionGuesser;
  private Supplier<Rotation2d> truthSupplier;

  // target position
  private double whereItWantsToGo;
  private double whereItStarted;
  private Distance zoomZoomSpeed;
  private final Supplier<Distance> magicDataSource;
  protected boolean hasReachedTarget = false;

  @SuppressWarnings("unused")
  private static final double FUDGE = 1.0;

  private double acceleration;
  private boolean controlY;
  private double controllerX;
  private double controllerY;

  public AxisAssist(
      Supplier<Pose2d> positionSupplier,
      Supplier<Rotation2d> yawSupplier,
      double targetPosition,
      boolean controlY) {
    super(yawSupplier);
    this.positionGuesser =
        () -> controlY ? positionSupplier.get().getX() : positionSupplier.get().getY();
    truthSupplier = () -> positionSupplier.get().getRotation();
    this.whereItWantsToGo = targetPosition;
    this.controlY = controlY;
    this.magicDataSource =
        () ->
            controlY
                ? RobotState.getInstance().getVelocity().getMeasureX()
                : RobotState.getInstance().getVelocity().getMeasureY();
    zoomZoomSpeed = magicDataSource.get();

    // setting up the ProfiledPIDController
    magicNumberBox =
        new ProfiledPIDController(
            PID_AUTOALIGN_CONSTANTS.kP(),
            PID_AUTOALIGN_CONSTANTS.kI(),
            PID_AUTOALIGN_CONSTANTS.kD(),
            new Constraints(
                PID_AUTOALIGN_CONSTANTS.maxVelocity(), PID_AUTOALIGN_CONSTANTS.maxAcceleration()),
            Constants.PERIODIC_LOOP_SEC);
    setTargetPosition(targetPosition);
    magicNumberBox.disableContinuousInput();
    magicNumberBox.setTolerance(0, 0);
  }

  /* accept driver input from joysticks */
  public void acceptJoystickInput(double controllerX, double controllerY, double acceleration) {
    this.controllerX = controllerX;
    this.controllerY = controllerY;
    this.acceleration = acceleration;
  }

  // calculate how to get to the desired position
  public void calculateLinearMovement() {
    double currToTargDx = positionGuesser.get() - whereItWantsToGo;

    double startToTargDx = whereItStarted - whereItWantsToGo;

    double startToCurrDx = whereItStarted - positionGuesser.get();

    // the naming is very important
    double magTranslCurrPos = startToCurrDx;
    double magTranslTargPos = startToTargDx;
    // can change to simpler varaibles above, and the problem being we use magnitude, so we combine
    // x and y, but we have to pslit them at a larger level
    double pidOutput = magicNumberBox.calculate(magTranslCurrPos, magTranslTargPos);
    double magVel = pidOutput + magicNumberBox.getSetpoint().velocity;
    magVel = (Math.abs(magVel) < AUTOALIGN_VELOCITY_DEADBAND ? 0 : magVel);

    // the gyro lies. always.
    zoomZoomSpeed = Units.Meters.of(magVel);
    if (Math.abs(positionGuesser.get() - whereItWantsToGo) < AUTOALIGN_POSITION_DEADBAND) {
      zoomZoomSpeed = Units.Meters.of(0);
    }

    Logger.recordOutput("Swerve/AxisAssist/SetpointPos", magicNumberBox.getSetpoint().position);
    Logger.recordOutput("Swerve/AxisAssist/CurrPos", magTranslCurrPos);
    Logger.recordOutput("Swerve/AxisAssist/TargPos", magTranslTargPos);
    Logger.recordOutput("Swerve/AxisAssist/MagVel", magVel);
    Logger.recordOutput("Swerve/AxisAssist/Target", whereItWantsToGo);
    Logger.recordOutput("Swerve/AxisAssist/TrapVel", magicNumberBox.getSetpoint().velocity);
    Logger.recordOutput("Swerve/AxisAssist/PIDVel", pidOutput);
  }

  public Distance calculateLinearVelocity(double y) {
    double magnitude = MathUtil.applyDeadband(Math.abs(y), 0.1);
    magnitude = Math.pow(magnitude, 1.5) * 3;
    if (RobotState.isAllianceRed()) {
      if (y < 0) {
        magnitude = magnitude * -1;
      }
    } else {
      if (y > 0) {
        magnitude = magnitude * -1;
      }
    }
    return Units.Meters.of(magnitude);
  }

  // update the values
  public ChassisSpeeds update() {
    calculateLinearMovement();
    Distance controlAxisVel = calculateLinearVelocity(controlY ? controllerY : controllerX);
    Logger.recordOutput("Swerve/AxisAssist/ControlAxisVel", controlAxisVel);
    Logger.recordOutput("Swerve/AxisAssist/PidAxisVel", zoomZoomSpeed);
    return ChassisSpeeds.fromFieldRelativeSpeeds(
        controlY ? zoomZoomSpeed.in(Units.Meters) : controlAxisVel.in(Units.Meters),
        controlY ? controlAxisVel.in(Units.Meters) : zoomZoomSpeed.in(Units.Meters),
        0,
        truthSupplier.get().plus(Rotation2d.k180deg));
  }

  // log your data in advantage kit
  public double getTargetPosition() {
    return whereItWantsToGo;
  }

  public Distance getXVel() {
    return Units.Meters.of(0).minus(controlY ? zoomZoomSpeed : Meters.of(0));
  }

  public Distance getYVel() {
    return Units.Meters.of(0).minus(!controlY ? zoomZoomSpeed : Meters.of(0));
  }

  // public Distance getYVel() {
  //   return Units.Meters.of(0).minus(yVel);
  // }

  public void setTargetPosition(double targetPosition) {
    whereItStarted = positionGuesser.get();
    this.whereItWantsToGo = targetPosition;
    double magTranslCurrPos = positionGuesser.get() - whereItStarted;
    double magTanslTargPos = targetPosition - whereItStarted;
    magicNumberBox.setGoal(magTanslTargPos);
    magicNumberBox.reset(magTranslCurrPos, calculateForwardVelocity().in(Units.Meters));
  }

  public Distance calculateForwardVelocity() {
    return zoomZoomSpeed;
  }

  public boolean atTarget() {
    return hasReachedTarget =
        Math.abs(positionGuesser.get() - whereItWantsToGo)
            < PID_AUTOALIGN_CONSTANTS.tolerance() * (hasReachedTarget ? 4 : 1);
  }

  private double getMaxLinearVelocity() {
    return DRIVE_CONFIG.maxLinearVelocity();
  }
}
